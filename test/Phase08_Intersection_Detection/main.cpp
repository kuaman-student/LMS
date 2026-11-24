#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ARC16.h>
#include "Config.h"

// ==================== FALLBACK ENCODER & KINEMATIC CONSTANTS ====================
#ifndef PIN_ENC_L_A
#define PIN_ENC_L_A 34
#define PIN_ENC_L_B 35
#define PIN_ENC_R_A 36
#define PIN_ENC_R_B 39
#endif

const float WHEEL_DIAMETER_MM = 34.0f;     // GA12-N20 standard wheel diameter (adjust if needed)
const float ENCODER_CPR       = 28.0f * 30.0f; // 4x quadrature CPR * gearbox ratio (update from Phase 03)
const float TICKS_PER_MM      = ENCODER_CPR / (PI * WHEEL_DIAMETER_MM);

const float INCH_FORWARD_MM   = 15.0f;     // Branch continuity verification distance
const float END_ZONE_MM       = 50.0f;     // Solid black finish pad confirmation distance
const float DEAD_END_MM       = 10.0f;     // White floor confirmation distance

// ==================== RUNTIME PARAMETERS ====================
float Kp          = 0.025f;
float Kd          = 0.180f;
int   basePwm     = 110;
int   steerSign   = 1;
bool  botRunning  = false;

// ==================== STATE MACHINE FOR INTERSECTION DETECTION ====================
enum TrackState {
    STATE_IDLE,
    STATE_TRACKING,
    STATE_VERIFY_BRANCH,
    STATE_VERIFY_ENDZONE,
    STATE_VERIFY_DEADEND,
    STATE_HALTED_AT_NODE
};

TrackState currentState = STATE_IDLE;
long  snapshotTicks     = 0;
bool  seenLeftWing      = false;
bool  seenRightWing     = false;
String lastJunctionName = "NONE";

// ==================== QUADRATURE ENCODERS (ISR) ====================
volatile long ticksLeft  = 0;
volatile long ticksRight = 0;

void IRAM_ATTR isrEncLeft() {
    if (digitalRead(PIN_ENC_L_A) == digitalRead(PIN_ENC_L_B)) ticksLeft++;
    else ticksLeft--;
}

void IRAM_ATTR isrEncRight() {
    if (digitalRead(PIN_ENC_R_A) == digitalRead(PIN_ENC_R_B)) ticksRight++;
    else ticksRight--;
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

// ==================== ARC-16 SENSOR & ZONE FLAGS ====================
ARC16 sensors;
int selectPins[4] = {PIN_MUX_S0, PIN_MUX_S1, PIN_MUX_S2, PIN_MUX_S3};
int rawValues[16];
int calMin[16];
int calMax[16];
int normValues[16];

struct ZoneSignature {
    bool leftWing;
    bool center;
    bool rightWing;
    int  activeCount;
    float lineErr;
};

float lastValidErr = 0.0f;
float prevErr      = 0.0f;
float currentErr   = 0.0f;

// ==================== TB6612FNG 20kHz PWM (ESP32 CORE v3.x) ====================
const int PWM_FREQ = 20000;
const int PWM_RES  = 8;

// ==================== UDP TELEMETRY & CLI ====================
WiFiUDP udp;
const uint16_t CMD_LISTEN_PORT = 8080;
const uint16_t BRIDGE_PORT     = 9000;
IPAddress      broadcastIP(192, 168, 4, 255);

void sendLog(const char* msg) {
    Serial.print(msg);
    udp.beginPacket(broadcastIP, BRIDGE_PORT);
    udp.write((uint8_t*)msg, strlen(msg));
    udp.endPacket();
}

// ==================== MOTOR HELPERS ====================
void setMotorLeft(int speed) {
    speed = constrain(speed, -255, 255);
    if (speed > 0) {
        digitalWrite(PIN_AIN1, HIGH);
        digitalWrite(PIN_AIN2, LOW);
        ledcWrite(PIN_PWMA, speed);
    } else if (speed < 0) {
        digitalWrite(PIN_AIN1, LOW);
        digitalWrite(PIN_AIN2, HIGH);
        ledcWrite(PIN_PWMA, -speed);
    } else {
        digitalWrite(PIN_AIN1, HIGH);
        digitalWrite(PIN_AIN2, HIGH);
        ledcWrite(PIN_PWMA, 0);
    }
}

void setMotorRight(int speed) {
    speed = constrain(speed, -255, 255);
    if (speed > 0) {
        digitalWrite(PIN_BIN1, HIGH);
        digitalWrite(PIN_BIN2, LOW);
        ledcWrite(PIN_PWMB, speed);
    } else if (speed < 0) {
        digitalWrite(PIN_BIN1, LOW);
        digitalWrite(PIN_BIN2, HIGH);
        ledcWrite(PIN_PWMB, -speed);
    } else {
        digitalWrite(PIN_BIN1, HIGH);
        digitalWrite(PIN_BIN2, HIGH);
        ledcWrite(PIN_PWMB, 0);
    }
}

void stopMotors() {
    setMotorLeft(0);
    setMotorRight(0);
}

// ==================== READ & CLASSIFY SENSOR ZONES ====================
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
        }
        if (val < 150) val = 0;
        normValues[i] = val;

        if (val > 600) active++;
        weightedSum += (long)i * 1000L * val;
        totalSignal += val;
    }

    ZoneSignature z;
    z.activeCount = active;

    // Zone thresholds: Left Wing (0..2), Center (6..9), Right Wing (13..15)
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

void haltAtJunction(const char* label, float travelMm) {
    stopMotors();
    botRunning = false;
    currentState = STATE_HALTED_AT_NODE;
    lastJunctionName = String(label);
    digitalWrite(PIN_LED3, LOW);

    char msg[128];
    snprintf(msg, sizeof(msg), "[JUNCTION] %s | Verified over %.1f mm | Send 'start' to continue\n",
             label, travelMm);
    sendLog(msg);
}

// ==================== 4-SECOND CALIBRATION ====================
void calibrateArray() {
    botRunning = false;
    currentState = STATE_IDLE;
    stopMotors();
    sendLog("[CAL] Sweeping ARC-16 for 4 seconds...\n");

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
    sendLog("[CAL] Complete!\n");
}

// ==================== CLI COMMAND HANDLER ====================
void executeCommand(String cmd) {
    cmd.trim();
    cmd.toLowerCase();
    if (cmd.length() == 0) return;

    char reply[128];
    if (cmd.startsWith("set kp "))      { Kp = cmd.substring(7).toFloat(); snprintf(reply, sizeof(reply), "[OK] Kp=%.5f\n", Kp); }
    else if (cmd.startsWith("set kd ")) { Kd = cmd.substring(7).toFloat(); snprintf(reply, sizeof(reply), "[OK] Kd=%.5f\n", Kd); }
    else if (cmd.startsWith("set base ")) { basePwm = constrain(cmd.substring(9).toInt(), 0, 255); snprintf(reply, sizeof(reply), "[OK] Base=%d\n", basePwm); }
    else if (cmd == "invert")           { steerSign = -steerSign; snprintf(reply, sizeof(reply), "[OK] SteerSign=%d\n", steerSign); }
    else if (cmd == "cal")              { calibrateArray(); return; }
    else if (cmd == "start" || cmd == "resume") {
        prevErr = 0.0f;
        seenLeftWing = false;
        seenRightWing = false;
        botRunning = true;
        currentState = STATE_TRACKING;
        digitalWrite(PIN_LED3, HIGH);
        snprintf(reply, sizeof(reply), "[RUN] Tracking until next junction (Base=%d)...\n", basePwm);
    }
    else if (cmd == "stop") {
        botRunning = false;
        currentState = STATE_IDLE;
        stopMotors();
        digitalWrite(PIN_LED3, LOW);
        snprintf(reply, sizeof(reply), "[HALT] Stopped!\n");
    }
    else if (cmd == "status") {
        snprintf(reply, sizeof(reply), "[STATUS] State:%d | LastNode:%s | Ticks:%ld | Kp:%.4f Kd:%.4f Base:%d\n",
                 currentState, lastJunctionName.c_str(), getAverageTicks(), Kp, Kd, basePwm);
    }
    else {
        snprintf(reply, sizeof(reply), "[ERR] Cmds: cal, start, resume, stop, invert, set kp/kd/base <v>, status\n");
    }
    sendLog(reply);
}

void setup() {
    Serial.begin(115200);
    delay(300);

    pinMode(PIN_LED3, OUTPUT);
    pinMode(PIN_KEY3, INPUT_PULLUP);
    digitalWrite(PIN_LED3, LOW);

    // Encoders setup
    pinMode(PIN_ENC_L_A, INPUT_PULLUP);
    pinMode(PIN_ENC_L_B, INPUT_PULLUP);
    pinMode(PIN_ENC_R_A, INPUT_PULLUP);
    pinMode(PIN_ENC_R_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrEncLeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrEncRight, CHANGE);

    // Motor driver setup (ESP32 Core v3.x)
    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_AIN2, OUTPUT);
    pinMode(PIN_BIN1, OUTPUT);
    pinMode(PIN_BIN2, OUTPUT);
    ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RES);
    ledcAttach(PIN_PWMB, PWM_FREQ, PWM_RES);
    stopMotors();

    // ARC-16 setup
    analogReadResolution(10);
    analogSetPinAttenuation(PIN_ARC16_SIG, ADC_11db);
    sensors.begin(selectPins, PIN_MUX_EN_DUMMY, PIN_ARC16_SIG);
    for (int i = 0; i < 16; i++) { calMin[i] = 100; calMax[i] = 900; }

    // Wi-Fi SoftAP
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
    udp.begin(CMD_LISTEN_PORT);

    Serial.printf("\n[PHASE 08] Intersection & Junction Signature Detector Ready!\n");
}

void loop() {
    // 1. Incoming UDP CLI commands
    int pktSize = udp.parsePacket();
    if (pktSize > 0) {
        char in[128];
        int len = udp.read(in, sizeof(in) - 1);
        if (len > 0) { in[len] = '\0'; executeCommand(String(in)); }
    }

    // 2. Physical KEY3 Start/Resume toggle
    if (digitalRead(PIN_KEY3) == LOW) {
        delay(30);
        if (digitalRead(PIN_KEY3) == LOW) {
            if (botRunning) executeCommand("stop");
            else executeCommand("start");
            while (digitalRead(PIN_KEY3) == LOW) delay(10);
        }
    }

    // 3. 500 Hz Control & Landmark State Machine
    static unsigned long lastLoopUs = 0;
    unsigned long nowUs = micros();
    if (nowUs - lastLoopUs >= 2000) {
        lastLoopUs = nowUs;

        ZoneSignature z = readZones();
        currentErr = z.lineErr;

        if (!botRunning) {
            stopMotors();
        } else {
            // PD Steering Calculation
            float derivative = currentErr - prevErr;
            prevErr = currentErr;
            float steer = steerSign * ((Kp * currentErr) + (Kd * derivative));

            // During branch verification, drive straight using center line (or zero steer if solid bar)
            if (currentState == STATE_VERIFY_BRANCH || currentState == STATE_VERIFY_ENDZONE) {
                steer = (z.activeCount >= 12) ? 0.0f : (steer * 0.4f);
            }

            int pwmL = constrain((int)(basePwm + steer), -255, 255);
            int pwmR = constrain((int)(basePwm - steer), -255, 255);
            setMotorLeft(pwmL);
            setMotorRight(pwmR);

            // Junction Detection State Machine
            switch (currentState) {
                case STATE_TRACKING:
                    if (z.activeCount >= 14) {
                        snapshotTicks = getAverageTicks();
                        seenLeftWing = true;
                        seenRightWing = true;
                        currentState = STATE_VERIFY_ENDZONE;
                    }
                    else if (z.leftWing || z.rightWing) {
                        snapshotTicks = getAverageTicks();
                        seenLeftWing  = z.leftWing;
                        seenRightWing = z.rightWing;
                        currentState  = STATE_VERIFY_BRANCH;
                    }
                    else if (z.activeCount == 0) {
                        snapshotTicks = getAverageTicks();
                        currentState  = STATE_VERIFY_DEADEND;
                    }
                    break;

                case STATE_VERIFY_BRANCH: {
                    // Latch any wing seen during the 15 mm inch-forward window
                    if (z.leftWing)  seenLeftWing = true;
                    if (z.rightWing) seenRightWing = true;

                    float dist = getDistanceDeltaMm(snapshotTicks);
                    if (dist >= INCH_FORWARD_MM) {
                        if (z.activeCount >= 14) {
                            currentState = STATE_VERIFY_ENDZONE;
                        } else if (seenLeftWing && seenRightWing && z.center) {
                            haltAtJunction("CROSS", dist);
                        } else if (seenLeftWing && seenRightWing && !z.center) {
                            haltAtJunction("T-JUNCTION", dist);
                        } else if (seenLeftWing && !seenRightWing && z.center) {
                            haltAtJunction("LEFT-STRAIGHT", dist);
                        } else if (seenLeftWing && !seenRightWing && !z.center) {
                            haltAtJunction("LEFT-ONLY", dist);
                        } else if (!seenLeftWing && seenRightWing && z.center) {
                            haltAtJunction("RIGHT-STRAIGHT", dist);
                        } else {
                            haltAtJunction("RIGHT-ONLY", dist);
                        }
                    }
                    break;
                }

                case STATE_VERIFY_ENDZONE: {
                    float dist = getDistanceDeltaMm(snapshotTicks);
                    if (z.activeCount < 12) {
                        // Crossed a perpendicular bar before 50 mm -> it's a CROSS or T-JUNCTION!
                        if (z.center) haltAtJunction("CROSS", dist);
                        else          haltAtJunction("T-JUNCTION", dist);
                    } else if (dist >= END_ZONE_MM) {
                        haltAtJunction("END-ZONE", dist);
                    }
                    break;
                }

                case STATE_VERIFY_DEADEND: {
                    float dist = getDistanceDeltaMm(snapshotTicks);
                    if (z.activeCount > 0) {
                        // False alarm / brief gap
                        currentState = STATE_TRACKING;
                    } else if (dist >= DEAD_END_MM) {
                        haltAtJunction("DEAD-END", dist);
                    }
                    break;
                }

                default:
                    break;
            }
        }
    }

    // 4. 20 Hz Teleplot Telemetry
    static unsigned long lastGraphMs = 0;
    if (millis() - lastGraphMs >= 50) {
        lastGraphMs = millis();
        char pkt[256];
        int pos = snprintf(pkt, sizeof(pkt),
                           ">err:%.1f\n>state:%d\n>ticks:%ld\n>kp:%.4f\n>kd:%.4f\n>base:%d\n",
                           currentErr, (int)currentState, getAverageTicks(), Kp, Kd, basePwm);
        udp.beginPacket(broadcastIP, BRIDGE_PORT);
        udp.write((uint8_t*)pkt, pos);
        udp.endPacket();
    }
}