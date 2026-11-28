#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "Config.h"

extern void onCommandReceived(String cmd);

namespace HybridComms {
    WiFiServer telnetServer(TELNET_PORT);
    WiFiClient telnetClient;
    WiFiUDP    udp;
    IPAddress  broadcastIP(192, 168, 4, 255);
    bool       enabled = true;

    inline void init(bool enableWifi = true) {
        enabled = enableWifi;
        if (!enabled) return;

        WiFi.mode(WIFI_AP);
        WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
        telnetServer.begin();
        telnetServer.setNoDelay(true);
    }

    inline void log(const char* msg) {
        Serial.print(msg);
        if (!enabled) return;

        if (telnetClient && telnetClient.connected()) {
            telnetClient.print(msg);
        }
        udp.beginPacket(broadcastIP, UDP_TELEMETRY_PORT);
        udp.write((uint8_t*)msg, strlen(msg));
        udp.endPacket();
    }

    inline void poll() {
        if (!enabled) return;

        if (telnetServer.hasClient()) {
            if (!telnetClient || !telnetClient.connected()) {
                if (telnetClient) telnetClient.stop();
                telnetClient = telnetServer.available();
                telnetClient.println("[CONNECTED] ProjectX Phase 10 Telnet CLI Ready.");
            } else {
                telnetServer.available().stop();
            }
        }

        if (telnetClient && telnetClient.connected() && telnetClient.available()) {
            String line = telnetClient.readStringUntil('\n');
            line.trim();
            if (line.length() > 0) onCommandReceived(line);
        }
    }

    inline void sendTelemetry(float err, float steer, int pwmL, int pwmR,
                              float yaw, float rateZ, const int* ir16, const char* pathStr) {
        if (!enabled) return;

        char pkt[320];
        int n = snprintf(pkt, sizeof(pkt),
            "TEL|%.1f|%.1f|%d|%d|%.2f|%.1f|0.00|0.00|1.00|%s|",
            err, steer, pwmL, pwmR, yaw, rateZ, (pathStr && pathStr[0]) ? pathStr : "-");

        for (int i = 0; i < 16; i++) {
            n += snprintf(pkt + n, sizeof(pkt) - n, "%d%s", ir16 ? ir16[i] : 0, (i < 15) ? "," : "\n");
        }

        udp.beginPacket(broadcastIP, UDP_TELEMETRY_PORT);
        udp.write((uint8_t*)pkt, n);
        udp.endPacket();
    }
}