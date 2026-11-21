#include <Arduino.h>
#include <ARC16.h>
#include "Config.h"

ARC16 sensors;

int selectPins[4] = {PIN_MUX_S0, PIN_MUX_S1, PIN_MUX_S2, PIN_MUX_S3};
int rawValues[16];
int calValues[16];

void runCalibrationSweep(uint32_t durationMs) {
    Serial.println("\n>>> [CALIBRATING] Sweep ARC-16 wide & slow across WHITE tape and BLACK floor...");
    digitalWrite(PIN_LED3, HIGH); // Solid LED indicates active calibration

    unsigned long startTime = millis();
    while (millis() - startTime < durationMs) {
        sensors.read(rawValues);
        sensors.calibrate();
        delay(2);
    }

    digitalWrite(PIN_LED3, LOW);
    Serial.println(">>> [DONE] Calibration Complete! Printing Min/Max Contrast Table:\n");

    int lowContrastCount = 0;
    for (int i = 0; i < 16; i++) {
        int minVal = sensors.getMin(i);
        int maxVal = sensors.getMax(i);
        int contrast = maxVal - minVal;
        if (contrast < 400) lowContrastCount++;

        Serial.printf("CH %02d | Min(White): %4d | Max(Black): %4d | Delta: %4d %s\n",
                      i, minVal, maxVal, contrast, (contrast < 400) ? "<-- CHECK HEIGHT!" : "[OK]");
    }

    if (lowContrastCount > 0) {
        Serial.printf("\n[WARNING] %d sensor(s) have low contrast! Check 3.5-4.5mm ground clearance.\n", lowContrastCount);
    } else {
        Serial.println("\n[SUCCESS] All 16 sensors calibrated cleanly!\n");
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(PIN_LED3, OUTPUT);
    pinMode(PIN_KEY3, INPUT_PULLUP);
    digitalWrite(PIN_LED3, LOW);

    // CRITICAL: Lock ESP32 ADC to 10-bit (0-1023) to match ARC16 library math
    analogReadResolution(10);
    analogSetPinAttenuation(PIN_ARC16_SIG, ADC_11db);

    // Initialize ARC16 library (Do NOT pass 'true' for external AREF on ESP32)
    sensors.begin(selectPins, PIN_MUX_EN_DUMMY, PIN_ARC16_SIG);
    sensors.setLine(WHITE);      // White tape on black arena floor
    sensors.setThreshold(500);   // 500/1000 threshold for digital on-line check

    Serial.println("===================================================");
    Serial.println(" [PHASE 02] ARC-16 Array Scan & Auto-Calibration");
    Serial.println("===================================================");
    Serial.println("Press KEY3 (GPIO 16) to start 5-second calibration sweep...");

    // Wait for KEY3 press before starting calibration
    while (digitalRead(PIN_KEY3) == HIGH) {
        digitalWrite(PIN_LED3, (millis() / 250) % 2); // Fast blink waiting for user
        delay(10);
    }
    delay(200); // Debounce

    runCalibrationSweep(5000);
}

void loop() {
    // Rule 1: Always call read() first in every loop iteration
    sensors.read(rawValues);
    sensors.readCalibrated(calValues);

    int pos       = sensors.readLine();     // -16000 (Right) to +16000 (Left)
    int err       = sensors.getError();     // Steering error from center (0)
    int onLineCnt = sensors.countOnLine();  // Total sensors seeing white line
    bool found    = sensors.lineFound();

    // Print Calibrated 16-Channel Bar (Index 15 = Leftmost -> Index 0 = Rightmost)
    Serial.print("L [");
    for (int i = 15; i >= 0; i--) {
        Serial.printf("%4d", calValues[i]);
        if (i > 0) Serial.print(" ");
    }
    Serial.printf("] R | Found:%d | Cnt:%2d | Pos:%6d | Err:%6d\n",
                  found, onLineCnt, pos, err);

    delay(50); // 20 Hz readable terminal stream (remove delay for 500 Hz race loop)
}