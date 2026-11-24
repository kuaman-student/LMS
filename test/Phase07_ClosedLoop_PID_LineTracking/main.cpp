#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ARC16.h>
#include "Config.h"

// ==================== RUNTIME TUNABLE PID PARAMETERS ====================
float Kp           = 0.025f;     // Start conservative at basePwm = 100
float Ki           = 0.000f;
float Kd           = 0.180f;
int   basePwm      = 100;        // Safe starting speed (tune up to 200+)
int   steerSign    = 1;          // Use 'invert' CLI cmd if bot steers away from line
bool  botRunning   = false;

// ==================== PID STATE VARIABLES ====================
float err          = 0.0f;
float prevErr      = 0.0f;
float integralErr  = 0.0f;
float steerOut     = 0.0f;
int   pwmLeft      = 0;
int   pwmRight     = 0;
float lastValidErr = 0.0f;

// ==================== ARC-16 SENSOR & CALIBRATION ====================
ARC16 sensors;
int selectPins[4] = {PIN_MUX_S0, PIN_MUX_S1, PIN_MUX_S2, PIN_MUX_S3};
int rawValues[16];
int calMin[16];
int calMax[16];
int normValues[16];

// ==================== TB6612FNG LEDC 20kHz SETTINGS ====================
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

// ==================== MOTOR CONTROL (WITH ACTIVE SHORT-BRAKE) ====================
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
        // Active short-brake
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
        // Active short-brake
        digitalWrite(PIN_BIN1, HIGH);
        digitalWrite(PIN_BIN2, HIGH);
        ledcWrite(PIN_PWMB, 0);
    }
}

void stopMotors() {
    pwmLeft = 0;
    pwmRight = 0;
    setMotorLeft(0);
    setMotorRight(0);
}

// ==================== 4-SECOND SENSOR CALIBRATION ====================
void calibrateArray() {
    botRunning = false;
    stopMotors();
    sendLog("[CAL] Sweep robot across line & floor for 4 seconds...\n");

    for (int i = 0; i < 16; i++) {
        calMin[i] = 1023;
        calMax[i] = 0;
    }

    unsigned long startMs = millis();
    while (millis() - startMs < 4000) {
        digitalWrite(PIN_LED3, (millis() / 100) % 2);
        sensors.read(rawValues);
        for (int i = 0; i < 16; i++) {
            if (rawValues[i] < calMin[i]) calMin[i] = rawValues[i];
            if (rawValues[i] > calMax[i]) calMax[i] = rawValues[i];
        }
        delay(5);
    }
    digitalWrite(PIN_LED3, LOW);

    int minContrast = 1023;
    for (int i = 0; i < 16; i++) {
        int span = calMax[i] - calMin[i];
        if (span < minContrast) minContrast = span;
    }
    char buf[96];
    snprintf(buf, sizeof(buf), "[CAL] Done! Min channel contrast = %d (Target > 300)\n", minContrast);
    sendLog(buf);
}

// ==================== WEIGHTED CENTROID LINE POSITION ====================
float computeLineError() {
    sensors.read(rawValues);
    long weightedSum = 0;
    long totalSignal = 0;

    for (int i = 0; i < 16; i++) {
        int span = calMax[i] - calMin[i];
        int val = 0;
        if (span > 50) {
            val = map(constrain(rawValues[i], calMin[i], calMax[i]), calMin[i], calMax[i], 0, 1000);
        }
        if (val < 150) val = 0; // Noise floor deadband
        normValues[i] = val;

        weightedSum += (long)i * 1000L * val;
        totalSignal += val;
    }

    if (totalSignal < 300) {
        // Line lost! Hold max steering toward last seen side
        return (lastValidErr >= 0) ? 7500.0f : -7500.0f;
    }

    float pos = (float)weightedSum / (float)totalSignal; // 0.0 to 15000.0
    float currentErr = pos - 7500.0f;                    // -7500.0 to +7500.0 (0 = dead center)
    lastValidErr = currentErr;
    return currentErr;
}

// ==================== WIRELESS CLI COMMAND PARSER ====================
void executeCommand(String cmd) {
    cmd.trim();
    cmd.toLowerCase();
    if (cmd.length() == 0) return;

    char reply[128];
    if (cmd.startsWith("set kp ")) {
        Kp = cmd.substring(7).toFloat();
        snprintf(reply, sizeof(reply), "[OK] Kp -> %.5f\n", Kp);
    }
    else if (cmd.startsWith("set ki ")) {
        Ki = cmd.substring(7).toFloat();
        integralErr = 0.0f;
        snprintf(reply, sizeof(reply), "[OK] Ki -> %.5f\n", Ki);
    }
    else if (cmd.startsWith("set kd ")) {
        Kd = cmd.substring(7).toFloat();
        snprintf(reply, sizeof(reply), "[OK] Kd -> %.5f\n", Kd);
    }
    else if (cmd.startsWith("set base ")) {
        basePwm = constrain(cmd.substring(9).toInt(), 0, 255);
        snprintf(reply, sizeof(reply), "[OK] Base PWM -> %d\n", basePwm);
    }
    else if (cmd == "invert") {
        steerSign = -steerSign;
        snprintf(reply, sizeof(reply), "[OK] Steer Sign -> %d\n", steerSign);
    }
    else if (cmd == "cal") {
        calibrateArray();
        return;
    }
    else if (cmd == "start") {
        integralErr = 0.0f;
        prevErr = 0.0f;
        botRunning = true;
        digitalWrite(PIN_LED3, HIGH);
        snprintf(reply, sizeof(reply), "[RUN] PID Tracking ACTIVE | Base=%d\n", basePwm);
    }
    else if (cmd == "stop") {
        botRunning = false;
        stopMotors();
        digitalWrite(PIN_LED3, LOW);
        snprintf(reply, sizeof(reply), "[HALT] Motors BRAKED!\n");
    }
    else if (cmd == "status") {
        snprintf(reply, sizeof(reply), "[STATUS] %s | Kp:%.4f Ki:%.4f Kd:%.4f | Base:%d | Sign:%d\n",
                 botRunning ? "RUN" : "STOP", Kp, Ki, Kd, basePwm, steerSign);
    }
    else {
        snprintf(reply, sizeof(reply), "[ERR] Cmds: cal, start, stop, invert, set kp/ki/kd/base <v>, status\n");
    }

    sendLog(reply);
}

void handleIncomingCommands() {
    int pktSize = udp.parsePacket();
    if (pktSize > 0) {
        char incoming[128];
        int len = udp.read(incoming, sizeof(incoming) - 1);
        if (len > 0) {
            incoming[len] = '\0';
            executeCommand(String(incoming));
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(300);

    pinMode(PIN_LED3, OUTPUT);
    pinMode(PIN_KEY3, INPUT_PULLUP);
    digitalWrite(PIN_LED3, LOW);

    // Motor pins & 20 kHz LEDC PWM setup (ESP32 Arduino Core v3.x API)
    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_AIN2, OUTPUT);
    pinMode(PIN_BIN1, OUTPUT);
    pinMode(PIN_BIN2, OUTPUT);
    ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RES);
    ledcAttach(PIN_PWMB, PWM_FREQ, PWM_RES);
    stopMotors();

    // ADC1_CH5 (GPIO 33) setup for ARC-16
    analogReadResolution(10);
    analogSetPinAttenuation(PIN_ARC16_SIG, ADC_11db);
    sensors.begin(selectPins, PIN_MUX_EN_DUMMY, PIN_ARC16_SIG);

    // Safe default calibration bounds (override with 'cal' command on arena)
    for (int i = 0; i < 16; i++) {
        calMin[i] = 100;
        calMax[i] = 900;
    }

    // Start SoftAP & UDP Server
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
    udp.begin(CMD_LISTEN_PORT);

    Serial.printf("\n[PHASE 07] Closed-Loop PID Ready | Wi-Fi: %s\n", WIFI_AP_SSID);
}

void loop() {
    // 1. Check incoming wireless CLI commands
    handleIncomingCommands();

    // 2. Physical KEY3 toggle (Short press = Start/Stop)
    if (digitalRead(PIN_KEY3) == LOW) {
        delay(30);
        if (digitalRead(PIN_KEY3) == LOW) {
            botRunning = !botRunning;
            if (!botRunning) stopMotors();
            digitalWrite(PIN_LED3, botRunning ? HIGH : LOW);
            char msg[64];
            snprintf(msg, sizeof(msg), "[KEY3] PID -> %s\n", botRunning ? "RUN" : "STOP");
            sendLog(msg);
            while (digitalRead(PIN_KEY3) == LOW) delay(10);
        }
    }

    // 3. Deterministic 500 Hz (2000 us) Closed-Loop PID Control
    static unsigned long lastPidUs = 0;
    unsigned long nowUs = micros();
    if (nowUs - lastPidUs >= 2000) {
        lastPidUs = nowUs;

        err = computeLineError();

        if (botRunning) {
            integralErr = constrain(integralErr + err, -10000.0f, 10000.0f);
            float derivative = err - prevErr;
            prevErr = err;

            steerOut = steerSign * ((Kp * err) + (Ki * integralErr) + (Kd * derivative));

            pwmLeft  = constrain((int)(basePwm + steerOut), -255, 255);
            pwmRight = constrain((int)(basePwm - steerOut), -255, 255);

            setMotorLeft(pwmLeft);
            setMotorRight(pwmRight);
        } else {
            stopMotors();
        }
    }

    // 4. Stream Teleplot Telemetry at 20 Hz (50 ms)
    static unsigned long lastGraphMs = 0;
    if (millis() - lastGraphMs >= 50) {
        lastGraphMs = millis();

        char pkt[256];
        int pos = snprintf(pkt, sizeof(pkt),
                           ">err:%.1f\n>steer:%.1f\n>pwmL:%d\n>pwmR:%d\n>kp:%.4f\n>kd:%.4f\n>base:%d\n",
                           err, steerOut, pwmLeft, pwmRight, Kp, Kd, basePwm);

        udp.beginPacket(broadcastIP, BRIDGE_PORT);
        udp.write((uint8_t*)pkt, pos);
        udp.endPacket();
    }
}