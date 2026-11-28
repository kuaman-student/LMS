#pragma once
#include <Arduino.h>

// ==================== 1. HARDWARE PINOUT (VERIFIED CONFLICT-FREE) ====================
#define PIN_KEY3                16      // Tactile Pushbutton (INPUT_PULLUP, Active LOW)
#define PIN_LED3                4       // Red Status Indicator LED (Active HIGH)

// ARC-16 IR Array & CD74HC4067 Multiplexer
#define PIN_MUX_S0              5
#define PIN_MUX_S1              18
#define PIN_MUX_S2              19
#define PIN_MUX_S3              23
#define PIN_ARC16_SIG           33      // ADC1_CH5 (Wi-Fi Safe)
#define PIN_MUX_EN_DUMMY        13      // Hardware EN tied to GND

// GA12-N20 Magnetic Encoders (Input-Only Pins)
#define PIN_ENC_L_A             39
#define PIN_ENC_L_B             36
#define PIN_ENC_R_A             34
#define PIN_ENC_R_B             35

// TB6612FNG Motor Driver (STBY tied to +3.3V)
#define PIN_PWMA                14
#define PIN_AIN1                26
#define PIN_AIN2                27
#define PIN_PWMB                32
#define PIN_BIN1                25
#define PIN_BIN2                17

// LSM6DSO 6-DOF IMU (Hardware Fast I2C @ 400kHz)
#define PIN_I2C_SDA             21
#define PIN_I2C_SCL             22
#define IMU_I2C_ADDR            0x6B

// ==================== 2. KINEMATICS & ARENA CONSTANTS ====================
#define WHEEL_DIAMETER_MM       30.0f
#define WHEEL_TRACK_MM          135.0f
#define GEAR_RATIO              30.0f
#define ENCODER_CPR             7.0f
#define TICKS_PER_MM            ((ENCODER_CPR * 4.0f * GEAR_RATIO) / (PI * WHEEL_DIAMETER_MM))

// Track Polarity: 0 = WHITE line on BLACK floor, 1 = BLACK line on WHITE floor
#define TRACK_LINE_WHITE        0
#define TRACK_LINE_BLACK        1
#define DEFAULT_LINE_MODE       TRACK_LINE_WHITE

#define INCH_FORWARD_MM         15.0f   // Branch continuity verification distance
#define END_ZONE_MM             50.0f   // Solid finish pad confirmation distance
#define DEAD_END_MM             10.0f   // Dead-end confirmation distance
#define WHEEL_CENTER_OFFSET_MM  35.0f   // Distance from ARC-16 bar to wheel axle

// ==================== 3. WIRELESS & PID/SPRINT PARAMETERS ====================
#define WIFI_AP_SSID            "LMS_PROJECTX_AP"
#define WIFI_AP_PASS            "projectx123"
#define TELNET_PORT             23
#define UDP_TELEMETRY_PORT      9000

// Exploration Gains (Smooth & Safe)
#define DEFAULT_KP              0.025f
#define DEFAULT_KI              0.000f
#define DEFAULT_KD              0.180f
#define EXPLORE_BASE_PWM        115     // Safe junction-detection speed

// Sprint Run Gains (High-Speed 1.5 - 1.83 m/s)
#define SPRINT_KP               0.032f
#define SPRINT_KD               0.260f
#define SPRINT_CRUISE_PWM       195     // High-speed base PWM
#define MAX_SPRINT_PWM          235     // Straight-corridor boost cap