#include <Arduino.h>
#include "Config.h"

// Non-blocking Debounce & Heartbeat Variables
const unsigned long DEBOUNCE_DELAY_MS = 40;
const unsigned long HEARTBEAT_MS      = 500;

int lastRawButtonState = HIGH;
int stableButtonState  = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long lastBlinkTime    = 0;
bool ledState = false;
uint32_t pressCount = 0;

void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(PIN_LED3, OUTPUT);
    pinMode(PIN_KEY3, INPUT_PULLUP);

    digitalWrite(PIN_LED3, LOW);

    Serial.println("\n==========================================");
    Serial.println(" [PHASE 01] Bare I/O & Boot Diagnostics");
    Serial.println("==========================================");
    Serial.printf("CPU Freq   : %d MHz\n", getCpuFrequencyMhz());
    Serial.printf("LED3 Pin   : GPIO %d\n", PIN_LED3);
    Serial.printf("KEY3 Pin   : GPIO %d (INPUT_PULLUP)\n", PIN_KEY3);
    Serial.println("Status     : Ready. Press KEY3 to test debounce...\n");
}

void loop() {
    unsigned long now = millis();

    // 1. Non-blocking Heartbeat LED (Confirms loop is alive & no watchdog freeze)
    if (now - lastBlinkTime >= HEARTBEAT_MS) {
        lastBlinkTime = now;
        ledState = !ledState;
        digitalWrite(PIN_LED3, ledState ? HIGH : LOW);
    }

    // 2. Non-blocking Switch Debounce for KEY3
    int currentRaw = digitalRead(PIN_KEY3);

    if (currentRaw != lastRawButtonState) {
        lastDebounceTime = now;
    }

    if ((now - lastDebounceTime) > DEBOUNCE_DELAY_MS) {
        if (currentRaw != stableButtonState) {
            stableButtonState = currentRaw;

            if (stableButtonState == LOW) {
                pressCount++;
                Serial.printf("[%lu ms] KEY3 PRESSED  (Total Presses: %lu)\n", now, pressCount);
                // Flash LED solid momentarily on press
                digitalWrite(PIN_LED3, HIGH);
            } else {
                Serial.printf("[%lu ms] KEY3 RELEASED\n", now);
            }
        }
    }

    lastRawButtonState = currentRaw;
}