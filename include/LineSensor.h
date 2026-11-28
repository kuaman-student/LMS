#pragma once
#include <Arduino.h>
#include <ARC16.h>
#include "Config.h"

struct ZoneSignature {
    bool  leftWing;
    bool  center;
    bool  rightWing;
    int   activeCount;
    float lineErr;
};

namespace LineSensor {
    ARC16 sensors;
    int selectPins[4] = {PIN_MUX_S0, PIN_MUX_S1, PIN_MUX_S2, PIN_MUX_S3};
    int rawValues[16];
    int calMin[16];
    int calMax[16];
    int normValues[16];
    float lastValidErr = 0.0f;

    inline void init() {
        analogReadResolution(10);
        analogSetPinAttenuation(PIN_ARC16_SIG, ADC_11db);
        sensors.begin(selectPins, PIN_MUX_EN_DUMMY, PIN_ARC16_SIG);

        for (int i = 0; i < 16; i++) {
            calMin[i] = 100;
            calMax[i] = 900;
        }
    }

    inline int calibrateSweep(uint32_t durationMs = 4000) {
        for (int i = 0; i < 16; i++) {
            calMin[i] = 1023;
            calMax[i] = 0;
        }
        unsigned long t0 = millis();
        while (millis() - t0 < durationMs) {
            digitalWrite(PIN_LED3, (millis() / 100) % 2);
            sensors.read(rawValues);
            for (int i = 0; i < 16; i++) {
                if (rawValues[i] < calMin[i]) calMin[i] = rawValues[i];
                if (rawValues[i] > calMax[i]) calMax[i] = rawValues[i];
            }
            delay(4);
        }
        digitalWrite(PIN_LED3, LOW);

        int minSpan = 1023;
        for (int i = 0; i < 16; i++) {
            int span = calMax[i] - calMin[i];
            if (span < minSpan) minSpan = span;
        }
        return minSpan;
    }

    inline ZoneSignature readZones() {
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
}