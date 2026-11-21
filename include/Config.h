#pragma once
#include <Arduino.h>

// ==================== 1. HARDWARE PINOUT (SHEET 3 VERIFIED) ====================
// User Interface
#define PIN_KEY3            16      // Tactile Pushbutton (INPUT_PULLUP, Active LOW)
#define PIN_LED3            4       // Red Status Indicator LED (Active HIGH)

// ARC-16 IR Array & CD74HC4067 Multiplexer
#define PIN_MUX_S0          5       // MUX Select Bit 0
#define PIN_MUX_S1          18      // MUX Select Bit 1
#define PIN_MUX_S2          19      // MUX Select Bit 2
#define PIN_MUX_S3          23      // MUX Select Bit 3
#define PIN_ARC16_SIG       33      // Analog Input (ADC1_CH5 - Wi-Fi Safe)
#define PIN_MUX_EN_DUMMY    13      // Hardware EN is tied to GND; dummy pin for ARC16 lib

// GA12-N20 Magnetic Encoders (Input-Only Pins)
#define PIN_ENC_L_A         39      // VN - Left Phase A (Interrupt ISR)
#define PIN_ENC_L_B         36      // VP - Left Phase B
#define PIN_ENC_R_A         34      // Right Phase A (Interrupt ISR)
#define PIN_ENC_R_B         35      // Right Phase B

// TB6612FNG Motor Driver (STBY tied to +3.3V)
#define PIN_PWMA            14      // Left Motor PWM (LEDC Ch 0)
#define PIN_AIN1            26      // Left Motor Direction 1
#define PIN_AIN2            27      // Left Motor Direction 2
#define PIN_PWMB            32      // Right Motor PWM (LEDC Ch 1)
#define PIN_BIN1            25      // Right Motor Direction 1
#define PIN_BIN2            17      // Right Motor Direction 2

// LSM6DSO 6-DOF IMU (Hardware Fast I2C @ 400kHz)
#define PIN_I2C_SDA         21      // Default Hardware SDA
#define PIN_I2C_SCL         22      // Default Hardware SCL
#define IMU_I2C_ADDR        0x6B    // Default LSM6DSO I2C Address (0x6B or 0x68)

// ==================== 2. KINEMATICS & ARENA CONSTANTS ====================
#define WHEEL_DIAMETER_MM   30.0f
#define WHEEL_TRACK_MM      135.0f
#define GEAR_RATIO          30.0f
#define ENCODER_CPR         7.0f
#define TICKS_PER_MM        ((ENCODER_CPR * 4.0f * GEAR_RATIO) / (PI * WHEEL_DIAMETER_MM))

// Track Polarity: 0 = WHITE line on BLACK floor, 1 = BLACK line on WHITE floor
#define TRACK_LINE_WHITE    0
#define TRACK_LINE_BLACK    1
#define DEFAULT_LINE_MODE   TRACK_LINE_WHITE
#define INCH_FORWARD_MM     15.0f