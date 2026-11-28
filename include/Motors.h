#pragma once
#include <Arduino.h>
#include "Config.h"

namespace Motors {
    constexpr uint32_t PWM_FREQ_HZ  = 20000;
    constexpr uint8_t  PWM_RES_BITS = 8;

    int trimL = 0;
    int trimR = 0;
    int lastPwmL = 0;
    int lastPwmR = 0;

    inline void stop() {
        lastPwmL = 0;
        lastPwmR = 0;
        digitalWrite(PIN_AIN1, HIGH);
        digitalWrite(PIN_AIN2, HIGH);
        ledcWrite(PIN_PWMA, 0);

        digitalWrite(PIN_BIN1, HIGH);
        digitalWrite(PIN_BIN2, HIGH);
        ledcWrite(PIN_PWMB, 0);
    }

    inline void init() {
        pinMode(PIN_AIN1, OUTPUT);
        pinMode(PIN_AIN2, OUTPUT);
        pinMode(PIN_BIN1, OUTPUT);
        pinMode(PIN_BIN2, OUTPUT);

        ledcAttach(PIN_PWMA, PWM_FREQ_HZ, PWM_RES_BITS);
        ledcAttach(PIN_PWMB, PWM_FREQ_HZ, PWM_RES_BITS);
        stop();
    }

    inline void setSpeeds(int leftSpeed, int rightSpeed) {
        if (leftSpeed != 0)  leftSpeed  += (leftSpeed > 0)  ? trimL : -trimL;
        if (rightSpeed != 0) rightSpeed += (rightSpeed > 0) ? trimR : -trimR;

        lastPwmL = constrain(leftSpeed,  -255, 255);
        lastPwmR = constrain(rightSpeed, -255, 255);

        if (lastPwmL > 0) {
            digitalWrite(PIN_AIN1, HIGH);
            digitalWrite(PIN_AIN2, LOW);
            ledcWrite(PIN_PWMA, lastPwmL);
        } else if (lastPwmL < 0) {
            digitalWrite(PIN_AIN1, LOW);
            digitalWrite(PIN_AIN2, HIGH);
            ledcWrite(PIN_PWMA, -lastPwmL);
        } else {
            digitalWrite(PIN_AIN1, HIGH);
            digitalWrite(PIN_AIN2, HIGH);
            ledcWrite(PIN_PWMA, 0);
        }

        if (lastPwmR > 0) {
            digitalWrite(PIN_BIN1, HIGH);
            digitalWrite(PIN_BIN2, LOW);
            ledcWrite(PIN_PWMB, lastPwmR);
        } else if (lastPwmR < 0) {
            digitalWrite(PIN_BIN1, LOW);
            digitalWrite(PIN_BIN2, HIGH);
            ledcWrite(PIN_PWMB, -lastPwmR);
        } else {
            digitalWrite(PIN_BIN1, HIGH);
            digitalWrite(PIN_BIN2, HIGH);
            ledcWrite(PIN_PWMB, 0);
        }
    }
}