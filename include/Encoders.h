#pragma once
#include <Arduino.h>
#include "Config.h"

namespace Encoders {
    volatile long ticksL = 0;
    volatile long ticksR = 0;

    const int8_t QEM_TABLE[16] = {
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0
    };

    volatile uint8_t prevStateL = 0;
    volatile uint8_t prevStateR = 0;

    void IRAM_ATTR isrLeft() {
        uint8_t curr = (digitalRead(PIN_ENC_L_A) << 1) | digitalRead(PIN_ENC_L_B);
        ticksL = ticksL + QEM_TABLE[(prevStateL << 2) | curr];
        prevStateL = curr;
    }

    void IRAM_ATTR isrRight() {
        uint8_t curr = (digitalRead(PIN_ENC_R_A) << 1) | digitalRead(PIN_ENC_R_B);
        ticksR = ticksR + QEM_TABLE[(prevStateR << 2) | curr];
        prevStateR = curr;
    }

    inline void init() {
        pinMode(PIN_ENC_L_A, INPUT);
        pinMode(PIN_ENC_L_B, INPUT);
        pinMode(PIN_ENC_R_A, INPUT);
        pinMode(PIN_ENC_R_B, INPUT);

        prevStateL = (digitalRead(PIN_ENC_L_A) << 1) | digitalRead(PIN_ENC_L_B);
        prevStateR = (digitalRead(PIN_ENC_R_A) << 1) | digitalRead(PIN_ENC_R_B);

        attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrLeft,  CHANGE);
        attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_B), isrLeft,  CHANGE);
        attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrRight, CHANGE);
        attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_B), isrRight, CHANGE);
    }

    inline void reset() {
        noInterrupts();
        ticksL = 0;
        ticksR = 0;
        interrupts();
    }

    inline long getAverageTicks() {
        noInterrupts();
        long avg = (labs(ticksL) + labs(ticksR)) / 2;
        interrupts();
        return avg;
    }

    inline float getDistanceDeltaMm(long snapshotTicks) {
        return (float)(getAverageTicks() - snapshotTicks) / TICKS_PER_MM;
    }
}