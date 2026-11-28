#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ARC16.h>
#include "Config.h"

// ==================== FALLBACK KINEMATIC CONSTANTS ====================
#ifndef END_ZONE_MM
#define END_ZONE_MM         50.0f
#endif
#ifndef DEAD_END_MM
#define DEAD_END_MM         10.0f
#endif
#ifndef WHEEL_CENTER_OFFSET_MM
#define WHEEL_CENTER_OFFSET_MM 35.0f // Distance from ARC-16 sensor bar to wheel axle
#endif

// ==================== LSM6DSO I2C REGISTERS ====================
#define LSM6DSO_WHO_AM_I    0x0F
#define LSM6DSO_CTRL1_XL    0x10
#define LSM6DSO_CTRL2_G     0x11
#define LSM6DSO_CTRL3_C     0x12
#define LSM6DSO_OUTZ_L_G    0x26
const float GYRO_SENS_DPS   = 0.070f; // ±2000 dps

// ==================== RUNTIME TUNABLE PARAMETERS ====================
float Kp          = DEFAULT_KP;
float Ki          = DEFAULT_KI;
float Kd          = DEFAULT_KD;
int   basePwm     = 115;       // Exploration speed (safe & accurate junction detection)
int   steerSign   = 1;
bool  botRunning  = false;

// ==================== LSRB EXPLORATION & TURN STATE MACHINE ====================
enum RobotState {
    STATE_IDLE,
    STATE_TRACKING,
    STATE_VERIFY_BRANCH,
    STATE_VERIFY_ENDZONE,
    STATE_VERIFY_DEADEND,
    STATE_ALIGN_AXLE,
    STATE_TURNING,
    STATE_MAZE_SOLVED
};

RobotState currentState = STATE_IDLE;
long  snapshotTicks     = 0;
bool  seenLeftWing      = false;
bool  seenRightWing     = false;
char  pendingTurnDir    = 'S'; // 'L', 'R', 'S', 'B'

// Turn Controller Variables
float targetTurnDeg     = 0.0f;
float turnYawAccumDeg   = 0.0f;
unsigned long lastGyroUs = 0;

// Path Recorder Buffer
char  rawPath[256];
int   pathLen           = 0;

// ==================== 4x QUADRATURE ENCODERS (QEM TABLE) ====================
volatile long ticksLeft  = 0;
volatile long ticksRight = 0;

const int8_t QEM_TABLE[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

volatile uint8_t prevLeftState  = 0;
volatile uint8_t prevRightState = 0;

void IRAM_ATTR isrEncLeft() {
    uint8_t a = digitalRead(PIN_ENC_L_A);
    uint8_t b = digitalRead(PIN_ENC_L_B);
    uint8_t curr = (a << 1) | b;
    ticksLeft = ticksLeft + QEM_TABLE[(prevLeftState << 2) | curr];
    prevLeftState = curr;
}

void IRAM_ATTR isrEncRight() {
    uint8_t a = digitalRead(PIN_ENC_R_A);
    uint8_t b = digitalRead(PIN_ENC_R_B);
    uint8_t curr = (a << 1) | b;
    ticksRight = ticksRight + QEM_TABLE[(prevRightState << 2) | curr];
    prevRightState = curr;
}

long getAverageTicks() {
    noInterrupts();
    long avg = (labs(ticksLeft) + labs(ticksRight)) / 2;
    interrupts();
    return avg;
}

float getDistanceDeltaMm(long refTicks) {
    return (float)(getAverageTicks() - refTicks) / TICKS_PER_MM;
}

// ==================== LSM6DSO IMU (400 kHz I2C) ====================
uint8_t activeImuAddr = IMU_I2C_ADDR;
float   gyroZBias     = 0.0f;
float   liveRateZ     = 0.0f;
float   totalYawDeg   = 0.0f;

void writeImuReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(activeImuAddr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

uint8_t readImuReg(uint8_t reg) {
    Wire.beginTransmission(activeImuAddr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(activeImuAddr, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0x00;
}

int16_t readGyroZRaw() {
    Wire.beginTransmission(activeImuAddr);
    Wire.write(LSM6DSO_OUTZ_L_G);
    Wire.endTransmission(false);
    Wire.requestFrom(activeImuAddr, (uint8_t)2);
    if (Wire.available() >= 2) {
        uint8_t lo = Wire.read();
        uint8_t hi = Wire.read();
        return (int16_t)((hi << 8) | lo);
    }
    return 0;
}

bool initIMU() {
    uint8_t who = readImuReg(LSM6DSO_WHO_AM_I);
    if (who != 0x6C) {
        activeImuAddr = 0x6A;
        who = readImuReg(LSM6DSO_WHO_AM_I);
    }
    if (who != 0x6C) return false;

    writeImuReg(LSM6DSO_CTRL3_C, 0x44); // BDU + IF_INC
    writeImuReg(LSM6DSO_CTRL2_G, 0x7C); // 833 Hz, ±2000 dps
    writeImuReg(LSM6DSO_CTRL1_XL, 0x00);
    return true;
}

void calibrateGyroBias(int samples = 600) {
    digitalWrite(PIN_LED3, HIGH);
    double sum = 0.0;
    for (int i = 0; i < samples; i++) {
        sum += (double)readGyroZRaw();
        delay(2);
    }
    gyroZBias = (float)(sum / samples);
    totalYawDeg = 0.0f;
    lastGyroUs = micros();
    digitalWrite(PIN_LED3, LOW);
}

void updateGyroYaw() {
    unsigned long nowUs = micros();
    float dt = (nowUs - lastGyroUs) / 1000000.0f;
    lastGyroUs = nowUs;

    float rate = ((float)readGyroZRaw() - gyroZBias) * GYRO_SENS_DPS;
    if (fabs(rate) < 0.15f) rate = 0.0f;
    liveRateZ = rate;
    totalYawDeg += rate * dt;
    turnYawAccumDeg += rate * dt;
}

// ==================== ARC-16 SENSOR & ZONE DETECTOR ====================
ARC16 sensors;
int selectPins[4] = {PIN_MUX_S0, PIN_MUX_S1, PIN_MUX_S2, PIN_MUX_S3};
int rawValues[16];
int calMin[16];
int calMax[16];
int normValues[16];

struct ZoneSignature {
    bool  leftWing;
    bool  center;
    bool  rightWing;
    int   activeCount;
    float lineErr;
};

float lastValidErr = 0.0f;
float prevErr      = 0.0f;
float integralErr  = 0.0f;
float currentErr   = 0.0f;
float currentSteer = 0.0f;
int   pwmL = 0, pwmR = 0;

ZoneSignature readZones() {
    sensors.read(rawValues);
    long weightedSum = 0;
    long totalSignal = 0;
    int active = 0;

    for (int i = 0; i < 16; i++) {
        int span = calMax[i] - calMin[i];
        int val = 0;
        if (span > 50) {
            val = map(constrain(rawValues[i], calMin[i], calMax[i]), calMin[i], calMax[i], 0, 1000);
            if (DEFAULT_LINE_MODE == TRACK_LINE_WHITE) val = 1000 - val;
        }
        if (val < 150) val = 0;
        normValues[i] = val;

        if (val > 600) active++;
        weightedSum += (long)i * 1000L * val;
        totalSignal += val;
    }

    ZoneSignature z;
    z.activeCount = active;

    int lCount = (normValues[0] > 600) + (normValues[1] > 600) + (normValues[2] > 600);
    int cCount = (normValues[6] > 600) + (normValues[7] > 600) + (normValues[8] > 600) + (normValues[9] > 600);
    int rCount = (normValues[13] > 600) + (normValues[14] > 600) + (normValues[15] > 600);

    z.leftWing  = (lCount >= 2);
    z.center    = (cCount >= 1);
    z.rightWing = (rCount >= 2);

    if (totalSignal < 300) {
        z.lineErr = (lastValidErr >= 0) ? 7500.0f : -7500.0f;
    } else {
        float pos = (float)weightedSum / (float)totalSignal;
        z.lineErr = pos - 7500.0f;
        lastValidErr = z.lineErr;
    }
    return z;
}

// ==================== TB6612FNG 20kHz MOTOR CONTROL ====================
const int PWM_FREQ = 20000;
const int PWM_RES  = 8;

void setMotors(int leftSpeed, int rightSpeed) {
    pwmL = constrain(leftSpeed,  -255, 255);
    pwmR = constrain(rightSpeed, -255, 255);

    if (pwmL > 0) {
        digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW);
        ledcWrite(PIN_PWMA, pwmL);
    } else if (pwmL < 0) {
        digitalWrite(PIN_AIN1, LOW);  digitalWrite(PIN_AIN2, HIGH);
        ledcWrite(PIN_PWMA, -pwmL);
    } else {
        digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, HIGH);
        ledcWrite(PIN_PWMA, 0);
    }

    if (pwmR > 0) {
        digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW);
        ledcWrite(PIN_PWMB, pwmR);
    } else if (pwmR < 0) {
        digitalWrite(PIN_BIN1, LOW);  digitalWrite(PIN_BIN2, HIGH);
        ledcWrite(PIN_PWMB, -pwmR);
    } else {
        digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, HIGH);
        ledcWrite(PIN_PWMB, 0);
    }
}

void stopMotors() {
    setMotors(0, 0);
}

// ==================== HYBRID TELNET (23) + UDP (9000) COMMS ====================
WiFiServer telnetServer(TELNET_PORT);
WiFiClient telnetClient;
WiFiUDP    udp;
const uint16_t UDP_PORT = 9000;
IPAddress      broadcastIP(192, 168, 4, 255);

void logMsg(const char* msg) {
    Serial.print(msg);
    if (telnetClient && telnetClient.connected()) {
        telnetClient.print(msg);
    }
    udp.beginPacket(broadcastIP, UDP_PORT);
    udp.write((uint8_t*)msg, strlen(msg));
    udp.endPacket();
}

void recordMove(char dir) {
    if (pathLen < (int)(sizeof(rawPath) - 1)) {
        rawPath[pathLen++] = dir;
        rawPath[pathLen]   = '\0';
    }
    char buf[96];
    snprintf(buf, sizeof(buf), "[LSRB] Move '%c' | Path: %s\n", dir, rawPath);
    logMsg(buf);
}

// ==================== LSRB DECISION & TURN INITIATION ====================
void initiateDecision(bool hasLeft, bool hasStraight, bool hasRight, bool isDeadEnd) {
    char chosen = 'S';

    if (isDeadEnd) {
        chosen = 'B';
    } else if (hasLeft) {
        chosen = 'L';
    } else if (hasStraight) {
        chosen = 'S';
    } else if (hasRight) {
        chosen = 'R';
    } else {
        chosen = 'B';
    }

    // Only record moves at actual intersections/dead-ends (not forced single-way corners if desired, or record all)
    // Standard LSRB records every junction & dead-end decision:
    if (isDeadEnd || (hasLeft + hasStraight + hasRight) >= 2) {
        recordMove(chosen);
    }

    pendingTurnDir = chosen;

    if (chosen == 'S') {
        prevErr = 0.0f;
        currentState = STATE_TRACKING;
        return;
    }

    // Align wheel axle over junction center before pivoting
    snapshotTicks = getAverageTicks();
    currentState = STATE_ALIGN_AXLE;
}

void startGyroPivot(char dir) {
    stopMotors();
    turnYawAccumDeg = 0.0f;
    lastGyroUs = micros();

    if (dir == 'L')      targetTurnDeg =  90.0f;
    else if (dir == 'R') targetTurnDeg = -90.0f;
    else                 targetTurnDeg = 180.0f; // 'B' U-Turn

    currentState = STATE_TURNING;
}

// ==================== 4-SECOND CALIBRATION ====================
void calibrateArray() {
    botRunning = false;
    currentState = STATE_IDLE;
    stopMotors();
    logMsg("[CAL] Sweep ARC-16 across white tape & black floor for 4s...\n");

    for (int i = 0; i < 16; i++) {
        calMin[i] = 1023;
        calMax[i] = 0;
    }
    unsigned long t0 = millis();
    while (millis() - t0 < 4000) {
        digitalWrite(PIN_LED3, (millis() / 100) % 2);
        sensors.read(rawValues);
        for (int i = 0; i < 16; i++) {
            if (rawValues[i] < calMin[i]) calMin[i] = rawValues[i];
            if (rawValues[i] > calMax[i]) calMax[i] = rawValues[i];
        }
        delay(5);
    }
    digitalWrite(PIN_LED3, LOW);
    logMsg("[CAL] Calibration Complete!\n");
}

// ==================== CLI COMMAND HANDLER ====================
void executeCommand(String cmd) {
    cmd.trim();
    cmd.toLowerCase();
    if (cmd.length() == 0) return;

    char reply[160];
    if (cmd.startsWith("set kp "))        { Kp = cmd.substring(7).toFloat(); snprintf(reply, sizeof(reply), "[OK] Kp=%.5f\n", Kp); }
    else if (cmd.startsWith("set ki "))   { Ki = cmd.substring(7).toFloat(); snprintf(reply, sizeof(reply), "[OK] Ki=%.5f\n", Ki); }
    else if (cmd.startsWith("set kd "))   { Kd = cmd.substring(7).toFloat(); snprintf(reply, sizeof(reply), "[OK] Kd=%.5f\n", Kd); }
    else if (cmd.startsWith("set base ")) { basePwm = constrain(cmd.substring(9).toInt(), 0, 255); snprintf(reply, sizeof(reply), "[OK] Base=%d\n", basePwm); }
    else if (cmd == "invert")             { steerSign = -steerSign; snprintf(reply, sizeof(reply), "[OK] SteerSign=%d\n", steerSign); }
    else if (cmd == "zeroyaw")            { totalYawDeg = 0.0f; snprintf(reply, sizeof(reply), "[OK] Yaw zeroed.\n"); }
    else if (cmd == "cal")                { calibrateArray(); return; }
    else if (cmd == "start") {
        pathLen = 0;
        rawPath[0] = '\0';
        prevErr = 0.0f;
        integralErr = 0.0f;
        botRunning = true;
        currentState = STATE_TRACKING;
        digitalWrite(PIN_LED3, HIGH);
        snprintf(reply, sizeof(reply), "[RUN] Phase 09 LSRB Maze Exploration STARTED!\n");
    }
    else if (cmd == "stop") {
        botRunning = false;
        currentState = STATE_IDLE;
        stopMotors();
        digitalWrite(PIN_LED3, LOW);
        snprintf(reply, sizeof(reply), "[HALT] Motors BRAKED!\n");
    }
    else if (cmd == "status") {
        snprintf(reply, sizeof(reply), "[STATUS] State:%d | Yaw:%.1f | Path:%s | Base:%d\n",
                 currentState, totalYawDeg, pathLen ? rawPath : "-", basePwm);
    }
    else {
        snprintf(reply, sizeof(reply), "[ERR] Cmds: cal, start, stop, invert, zeroyaw, set kp/ki/kd/base <v>, status\n");
    }
    logMsg(reply);
}

void setup() {
    Serial.begin(115200);
    delay(300);

    pinMode(PIN_LED3, OUTPUT);
    pinMode(PIN_KEY3, INPUT_PULLUP);
    digitalWrite(PIN_LED3, LOW);

    // 1. 4x Quadrature Encoders
    pinMode(PIN_ENC_L_A, INPUT);
    pinMode(PIN_ENC_L_B, INPUT);
    pinMode(PIN_ENC_R_A, INPUT);
    pinMode(PIN_ENC_R_B, INPUT);
    prevLeftState  = (digitalRead(PIN_ENC_L_A) << 1) | digitalRead(PIN_ENC_L_B);
    prevRightState = (digitalRead(PIN_ENC_R_A) << 1) | digitalRead(PIN_ENC_R_B);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrEncLeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_B), isrEncLeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrEncRight, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_B), isrEncRight, CHANGE);

    // 2. TB6612FNG 20 kHz PWM
    pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
    pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
    ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RES);
    ledcAttach(PIN_PWMB, PWM_FREQ, PWM_RES);
    stopMotors();

    // 3. ARC-16 Sensor Array
    analogReadResolution(10);
    analogSetPinAttenuation(PIN_ARC16_SIG, ADC_11db);
    sensors.begin(selectPins, PIN_MUX_EN_DUMMY, PIN_ARC16_SIG);
    for (int i = 0; i < 16; i++) { calMin[i] = 100; calMax[i] = 900; }

    // 4. LSM6DSO IMU (400 kHz I2C)
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000);
    if (!initIMU()) {
        Serial.println("[ERROR] LSM6DSO IMU not found on SDA(21)/SCL(22)!");
        while (true) { digitalWrite(PIN_LED3, !digitalRead(PIN_LED3)); delay(150); }
    }
    calibrateGyroBias(600);

    // 5. Hybrid Wi-Fi SoftAP + Telnet (23) + UDP (9000)
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
    telnetServer.begin();
    telnetServer.setNoDelay(true);

    Serial.println("\n[PHASE 09] Gyro Pivot Turns + LSRB Maze Exploration Ready!");
}

void loop() {
    // 1. Telnet Client & Command Handling
    if (telnetServer.hasClient()) {
        if (!telnetClient || !telnetClient.connected()) {
            if (telnetClient) telnetClient.stop();
            telnetClient = telnetServer.available();
            telnetClient.println("[CONNECTED] ProjectX Phase 09 Telnet CLI Ready.");
        } else {
            telnetServer.available().stop();
        }
    }
    if (telnetClient && telnetClient.connected() && telnetClient.available()) {
        String line = telnetClient.readStringUntil('\n');
        executeCommand(line);
    }

    // 2. Standalone Physical KEY3 Operation (Short press = Start/Stop, Long press 1.5s = Calibrate)
    if (digitalRead(PIN_KEY3) == LOW) {
        unsigned long pressStart = millis();
        while (digitalRead(PIN_KEY3) == LOW) {
            if (millis() - pressStart > 1500) {
                calibrateArray();
                while (digitalRead(PIN_KEY3) == LOW) delay(10);
                return;
            }
            delay(10);
        }
        if (millis() - pressStart > 40) {
            if (botRunning) executeCommand("stop");
            else            executeCommand("start");
        }
    }

    // 3. Deterministic 500 Hz Control, Gyro Integration & LSRB State Machine
    static unsigned long lastLoopUs = 0;
    unsigned long nowUs = micros();
    if (nowUs - lastLoopUs >= 2000) {
        lastLoopUs = nowUs;

        updateGyroYaw();
        ZoneSignature z = readZones();
        currentErr = z.lineErr;

        if (!botRunning) {
            stopMotors();
        } else {
            switch (currentState) {
                case STATE_TRACKING: {
                    // Standard 500 Hz PID line following
                    integralErr = constrain(integralErr + currentErr, -10000.0f, 10000.0f);
                    float derivative = currentErr - prevErr;
                    prevErr = currentErr;
                    currentSteer = steerSign * ((Kp * currentErr) + (Ki * integralErr) + (Kd * derivative));

                    setMotors(basePwm + currentSteer, basePwm - currentSteer);

                    // Check for intersection / end-zone / dead-end triggers
                    if (z.activeCount >= 14) {
                        snapshotTicks = getAverageTicks();
                        seenLeftWing = true;
                        seenRightWing = true;
                        currentState = STATE_VERIFY_ENDZONE;
                    } else if (z.leftWing || z.rightWing) {
                        snapshotTicks = getAverageTicks();
                        seenLeftWing  = z.leftWing;
                        seenRightWing = z.rightWing;
                        currentState  = STATE_VERIFY_BRANCH;
                    } else if (z.activeCount == 0) {
                        snapshotTicks = getAverageTicks();
                        currentState  = STATE_VERIFY_DEADEND;
                    }
                    break;
                }

                case STATE_VERIFY_BRANCH: {
                    if (z.leftWing)  seenLeftWing = true;
                    if (z.rightWing) seenRightWing = true;

                    // Drive straight gently while verifying branch continuity
                    float steer = (z.activeCount >= 12) ? 0.0f : (steerSign * Kp * currentErr * 0.4f);
                    setMotors(basePwm + steer, basePwm - steer);

                    float dist = getDistanceDeltaMm(snapshotTicks);
                    if (dist >= INCH_FORWARD_MM) {
                        if (z.activeCount >= 14) {
                            currentState = STATE_VERIFY_ENDZONE;
                        } else {
                            // Classify exact junction and trigger LSRB decision
                            initiateDecision(seenLeftWing, z.center, seenRightWing, false);
                        }
                    }
                    break;
                }

                case STATE_VERIFY_ENDZONE: {
                    setMotors(basePwm, basePwm);
                    float dist = getDistanceDeltaMm(snapshotTicks);
                    if (z.activeCount < 12) {
                        // Crossed a perpendicular bar -> CROSS or T-JUNCTION
                        initiateDecision(true, z.center, true, false);
                    } else if (dist >= END_ZONE_MM) {
                        // Confirmed End-Zone Goal Pad!
                        stopMotors();
                        botRunning = false;
                        currentState = STATE_MAZE_SOLVED;
                        digitalWrite(PIN_LED3, LOW);
                        char msg[200];
                        snprintf(msg, sizeof(msg), "\n[GOAL] MAZE EXPLORED! Raw Path (%d moves): %s\n", pathLen, rawPath);
                        logMsg(msg);
                    }
                    break;
                }

                case STATE_VERIFY_DEADEND: {
                    setMotors(basePwm * 0.8f, basePwm * 0.8f);
                    float dist = getDistanceDeltaMm(snapshotTicks);
                    if (z.activeCount > 0) {
                        currentState = STATE_TRACKING;
                    } else if (dist >= DEAD_END_MM) {
                        initiateDecision(false, false, false, true); // U-Turn ('B')
                    }
                    break;
                }

                case STATE_ALIGN_AXLE: {
                    // Advance remaining distance so wheel axle sits directly over the intersection center
                    float remainingAlignMm = max(0.0f, WHEEL_CENTER_OFFSET_MM - INCH_FORWARD_MM);
                    setMotors(basePwm * 0.8f, basePwm * 0.8f);

                    if (getDistanceDeltaMm(snapshotTicks) >= remainingAlignMm) {
                        startGyroPivot(pendingTurnDir);
                    }
                    break;
                }

                case STATE_TURNING: {
                    float remainingDeg = targetTurnDeg - turnYawAccumDeg;
                    float absRem = fabs(remainingDeg);

                    // Proportional turn speed with minimum static-friction floor (PWM 75..150)
                    int turnPwm = constrain((int)(absRem * 2.2f + 65.0f), 75, 150);
                    int dirSign = (remainingDeg >= 0) ? 1 : -1;

                    // Counter-rotate wheels for zero-radius pivot turn
                    // Note: Verify in dry test that +dirSign rotates CCW (+Yaw)
                    setMotors(-dirSign * turnPwm, dirSign * turnPwm);

                    // Exit condition: Within 12° of target AND center sensor catches the new line (or within 2.5° deadband)
                    bool lineCaught = (absRem < 18.0f) && z.center;
                    if (lineCaught || absRem <= 2.5f) {
                        stopMotors();
                        prevErr = 0.0f;
                        integralErr = 0.0f;
                        currentState = STATE_TRACKING;
                    }
                    break;
                }

                default:
                    stopMotors();
                    break;
            }
        }
    }

    // 4. 20 Hz Hybrid UDP Telemetry Stream (Compatible with cockpit.py)
    static unsigned long lastTelMs = 0;
    if (millis() - lastTelMs >= 50) {
        lastTelMs = millis();
        char pkt[320];
        int n = snprintf(pkt, sizeof(pkt),
            "TEL|%.1f|%.1f|%d|%d|%.2f|%.1f|0.00|0.00|1.00|%s|",
            currentErr, currentSteer, pwmL, pwmR, totalYawDeg, liveRateZ, pathLen ? rawPath : "-");
        for (int i = 0; i < 16; i++) {
            n += snprintf(pkt + n, sizeof(pkt) - n, "%d%s", normValues[i], (i < 15) ? "," : "\n");
        }
        udp.beginPacket(broadcastIP, UDP_PORT);
        udp.write((uint8_t*)pkt, n);
        udp.endPacket();
    }
}