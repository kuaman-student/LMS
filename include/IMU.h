#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "Config.h"

namespace IMU {
    constexpr uint8_t REG_WHO_AM_I   = 0x0F;
    constexpr uint8_t REG_CTRL1_XL   = 0x10;
    constexpr uint8_t REG_CTRL2_G    = 0x11;
    constexpr uint8_t REG_CTRL3_C    = 0x12;
    constexpr uint8_t REG_OUTZ_L_G   = 0x26;
    constexpr float   GYRO_SENS_DPS  = 0.070f; // ±2000 dps

    uint8_t addr          = IMU_I2C_ADDR;
    float   gyroZBias     = 0.0f;
    float   liveRateZ     = 0.0f;
    float   totalYawDeg   = 0.0f;
    float   turnYawDeg    = 0.0f;
    unsigned long lastUs  = 0;

    inline void writeReg(uint8_t reg, uint8_t val) {
        Wire.beginTransmission(addr);
        Wire.write(reg);
        Wire.write(val);
        Wire.endTransmission();
    }

    inline uint8_t readReg(uint8_t reg) {
        Wire.beginTransmission(addr);
        Wire.write(reg);
        Wire.endTransmission(false);
        Wire.requestFrom(addr, (uint8_t)1);
        return Wire.available() ? Wire.read() : 0x00;
    }

    inline int16_t readGyroZRaw() {
        Wire.beginTransmission(addr);
        Wire.write(REG_OUTZ_L_G);
        Wire.endTransmission(false);
        Wire.requestFrom(addr, (uint8_t)2);
        if (Wire.available() >= 2) {
            uint8_t lo = Wire.read();
            uint8_t hi = Wire.read();
            return (int16_t)((hi << 8) | lo);
        }
        return 0;
    }

    inline bool init() {
        Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
        Wire.setClock(400000);

        uint8_t who = readReg(REG_WHO_AM_I);
        if (who != 0x6C) {
            addr = 0x6A;
            who = readReg(REG_WHO_AM_I);
        }
        if (who != 0x6C) return false;

        writeReg(REG_CTRL3_C,  0x44); // BDU + IF_INC
        writeReg(REG_CTRL2_G,  0x7C); // 833 Hz, ±2000 dps
        writeReg(REG_CTRL1_XL, 0x00); // Accel off for max I2C speed
        return true;
    }

    inline void calibrateBias(int samples = 600) {
        digitalWrite(PIN_LED3, HIGH);
        double sum = 0.0;
        for (int i = 0; i < samples; i++) {
            sum += (double)readGyroZRaw();
            delay(2);
        }
        gyroZBias   = (float)(sum / samples);
        totalYawDeg = 0.0f;
        turnYawDeg  = 0.0f;
        lastUs      = micros();
        digitalWrite(PIN_LED3, LOW);
    }

    inline void resetTurnYaw() {
        turnYawDeg = 0.0f;
        lastUs     = micros();
    }

    inline void update() {
        unsigned long nowUs = micros();
        float dt = (nowUs - lastUs) / 1000000.0f;
        lastUs = nowUs;

        float rate = ((float)readGyroZRaw() - gyroZBias) * GYRO_SENS_DPS;
        if (fabs(rate) < 0.15f) rate = 0.0f;

        liveRateZ    = rate;
        totalYawDeg += rate * dt;
        turnYawDeg  += rate * dt;
    }
}