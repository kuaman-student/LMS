#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ARC16.h>
#include "Config.h"

// ==================== RUNTIME TUNABLE PARAMETERS ====================
float Kp         = DEFAULT_KP;
float Ki         = DEFAULT_KI;
float Kd         = DEFAULT_KD;
int   basePwm    = DEFAULT_BASE_PWM;
bool  botRunning = false;

// ==================== ARC-16 SENSOR OBJECT ====================
ARC16 sensors;
int selectPins[4] = {PIN_MUX_S0, PIN_MUX_S1, PIN_MUX_S2, PIN_MUX_S3};
int rawValues[16];

// ==================== UDP TELEMETRY & CLI SERVER ====================
WiFiUDP udp;
const uint16_t CMD_LISTEN_PORT = 8080;               // Laptop se commands lene ke liye
const uint16_t BRIDGE_PORT     = 9000;               // Laptop ke bridge.py ko data bhejne ke liye
IPAddress      broadcastIP(192, 168, 4, 255);        // Sabhi connected Wi-Fi clients ko bhejega

void sendLog(const char* msg) {
    Serial.print(msg);
    udp.beginPacket(broadcastIP, BRIDGE_PORT);
    udp.write((uint8_t*)msg, strlen(msg));
    udp.endPacket();
}

// ==================== COMMAND PARSER ====================
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
    else if (cmd == "start") {
        botRunning = true;
        digitalWrite(PIN_LED3, HIGH);
        snprintf(reply, sizeof(reply), "[RUN] Bot ARMED / Running!\n");
    }
    else if (cmd == "stop") {
        botRunning = false;
        digitalWrite(PIN_LED3, LOW);
        snprintf(reply, sizeof(reply), "[HALT] EMERGENCY STOP!\n");
    }
    else if (cmd == "status") {
        snprintf(reply, sizeof(reply), "[STATUS] State:%s | Kp:%.4f | Ki:%.4f | Kd:%.4f | Base:%d | Heap:%u\n",
                 botRunning ? "RUN" : "STOP", Kp, Ki, Kd, basePwm, ESP.getFreeHeap());
    }
    else {
        snprintf(reply, sizeof(reply), "[ERR] Cmds: set kp/ki/kd/base <v>, start, stop, status\n");
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

    // Initialize ARC-16 on ADC1_CH5 (GPIO 33 - Wi-Fi Safe!)
    analogReadResolution(10);
    analogSetPinAttenuation(PIN_ARC16_SIG, ADC_11db);
    sensors.begin(selectPins, PIN_MUX_EN_DUMMY, PIN_ARC16_SIG);
    sensors.setLine(WHITE);
    sensors.setThreshold(500);

    // Start ESP32 SoftAP & UDP Listener
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
    udp.begin(CMD_LISTEN_PORT);

    Serial.printf("\n[PHASE 06] SoftAP '%s' Ready | IP: %s\n",
                  WIFI_AP_SSID, WiFi.softAPIP().toString().c_str());
}

void loop() {
    // 1. Handle wireless tuning & motion commands
    handleIncomingCommands();

    // 2. Scan ARC-16 array
    sensors.read(rawValues);
    int lineErr = sensors.getError();

    // 3. Physical KEY3 emergency toggle
    if (digitalRead(PIN_KEY3) == LOW) {
        botRunning = !botRunning;
        digitalWrite(PIN_LED3, botRunning ? HIGH : LOW);
        char msg[64];
        snprintf(msg, sizeof(msg), "[KEY3] State -> %s\n", botRunning ? "RUN" : "STOP");
        sendLog(msg);
        delay(250);
    }

    // 4. Stream Teleplot Variables at 20 Hz (strict >name:value\n format, NO \r)
    static unsigned long lastGraphMs = 0;
    if (millis() - lastGraphMs >= 50) {
        lastGraphMs = millis();

        // Packet A: Channels 0-7
        char pktA[180];
        int posA = 0;
        for (int i = 0; i < 8; i++) {
            posA += snprintf(pktA + posA, sizeof(pktA) - posA, ">ch%d:%d\n", i, rawValues[i]);
        }
        udp.beginPacket(broadcastIP, BRIDGE_PORT);
        udp.write((uint8_t*)pktA, posA);
        udp.endPacket();

        // Packet B: Channels 8-15 + PID variables
        char pktB[220];
        int posB = 0;
        for (int i = 8; i < 16; i++) {
            posB += snprintf(pktB + posB, sizeof(pktB) - posB, ">ch%d:%d\n", i, rawValues[i]);
        }
        posB += snprintf(pktB + posB, sizeof(pktB) - posB, ">err:%d\n>kp:%.4f\n>ki:%.4f\n>kd:%.4f\n>base:%d\n",
                         lineErr, Kp, Ki, Kd, basePwm);
        udp.beginPacket(broadcastIP, BRIDGE_PORT);
        udp.write((uint8_t*)pktB, posB);
        udp.endPacket();
    }
}