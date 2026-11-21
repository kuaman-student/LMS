#include <Arduino.h>
#include <Wire.h>
#include "Config.h"

// ==================== LSM6DSO REGISTER MAP ====================
#define LSM6DSO_WHO_AM_I    0x0F    // Expected response: 0x6C
#define LSM6DSO_CTRL1_XL    0x10    // Accelerometer control
#define LSM6DSO_CTRL2_G     0x11    // Gyroscope control (ODR & Full-Scale)
#define LSM6DSO_CTRL3_C     0x12    // Block Data Update (BDU) & Auto-Increment
#define LSM6DSO_OUTZ_L_G    0x26    // Gyroscope Z-axis Low Byte (Yaw rate)

// Sensitivity for ±2000 dps full-scale (70 mdps/LSB = 0.070 dps/LSB)
const float GYRO_SENSITIVITY_DPS = 0.070f;

uint8_t activeImuAddr = IMU_I2C_ADDR; // Default 0x6B from Config.h
float gyroZBias       = 0.0f;
float currentYawDeg   = 0.0f;
unsigned long lastMicros = 0;

// ==================== LOW-LEVEL I2C HELPERS ====================
void writeRegister(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(activeImuAddr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg) {
    Wire.beginTransmission(activeImuAddr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(activeImuAddr, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0x00;
}

int16_t readGyroZRaw() {
    Wire.beginTransmission(activeImuAddr);
    Wire.write(LSM6DSO_OUTZ_L_G);
    Wire.endTransmission(false);
    Wire.requestFrom(activeImuAddr, (uint8_t)2);

    if (Wire.available() >= 2) {
        uint8_t lowByte  = Wire.read();
        uint8_t highByte = Wire.read();
        return (int16_t)((highByte << 8) | lowByte);
    }
    return 0;
}

// ==================== INITIALIZATION & CALIBRATION ====================
bool initLSM6DSO() {
    // Auto-detect address (0x6B if SA0=HIGH, 0x6A if SA0=LOW)
    uint8_t whoAmI = readRegister(LSM6DSO_WHO_AM_I);
    if (whoAmI != 0x6C) {
        activeImuAddr = 0x6A;
        whoAmI = readRegister(LSM6DSO_WHO_AM_I);
    }

    if (whoAmI != 0x6C) {
        Serial.printf("[ERROR] LSM6DSO not found! WHO_AM_I returned: 0x%02X\n", whoAmI);
        return false;
    }

    // CTRL3_C: Enable Block Data Update (BDU=1) + Register Auto-Increment (IF_INC=1) -> 0x44
    writeRegister(LSM6DSO_CTRL3_C, 0x44);

    // CTRL2_G: ODR = 833 Hz (0111) + Full-Scale = ±2000 dps (1100) -> 0x7C
    writeRegister(LSM6DSO_CTRL2_G, 0x7C);

    // CTRL1_XL: Power-down Accelerometer (0x00) to save I2C bandwidth (we only need Yaw Gyro)
    writeRegister(LSM6DSO_CTRL1_XL, 0x00);

    Serial.printf("[OK] LSM6DSO detected at I2C Address 0x%02X (WHO_AM_I = 0x6C)\n", activeImuAddr);
    return true;
}

void calibrateGyroBias(int samples = 1000) {
    Serial.println(">>> [CALIBRATING] Keep robot completely STILL on flat surface...");
    digitalWrite(PIN_LED3, HIGH);

    double sumZ = 0.0;
    for (int i = 0; i < samples; i++) {
        sumZ += (double)readGyroZRaw();
        delay(2); // ~2 seconds total for 1000 samples
    }

    gyroZBias = (float)(sumZ / samples);
    currentYawDeg = 0.0f;
    lastMicros = micros();

    digitalWrite(PIN_LED3, LOW);
    Serial.printf(">>> [DONE] Stationary Z-Bias Offset = %.2f LSB (%.3f dps)\n\n",
                  gyroZBias, gyroZBias * GYRO_SENSITIVITY_DPS);
}

void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(PIN_LED3, OUTPUT);
    pinMode(PIN_KEY3, INPUT_PULLUP);
    digitalWrite(PIN_LED3, LOW);

    // Start Hardware I2C on GPIO 21 (SDA) and GPIO 22 (SCL) at 400 kHz Fast Mode
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000);

    Serial.println("\n===================================================");
    Serial.println(" [PHASE 04] LSM6DSO Fast I2C & Yaw Gyro Integration");
    Serial.println("===================================================");

    if (!initLSM6DSO()) {
        Serial.println("[HALT] Check SDA(21), SCL(22), 3.3V, and GND wiring!");
        while (true) {
            digitalWrite(PIN_LED3, !digitalRead(PIN_LED3));
            delay(150);
        }
    }

    calibrateGyroBias(1000);
    Serial.println("Instructions: Rotate bot flat 90° or 180°. Press KEY3 to zero Yaw.\n");
}

void loop() {
    // Zero Yaw angle on KEY3 press
    if (digitalRead(PIN_KEY3) == LOW) {
        currentYawDeg = 0.0f;
        lastMicros = micros();
        digitalWrite(PIN_LED3, HIGH);
        Serial.println("\n>>> [RESET] Yaw Angle Zeroed to 0.00°!\n");
        delay(250);
        digitalWrite(PIN_LED3, LOW);
    }

    // 1. Measure exact time step dt in seconds
    unsigned long nowMicros = micros();
    float dt = (nowMicros - lastMicros) / 1000000.0f;
    lastMicros = nowMicros;

    // 2. Read raw Z-axis angular velocity & subtract stationary bias
    int16_t rawZ = readGyroZRaw();
    float correctedZ = (float)rawZ - gyroZBias;

    // Deadband filter: ignore tiny thermal noise (< 0.15 dps) when stationary
    float omegaZ_dps = correctedZ * GYRO_SENSITIVITY_DPS;
    if (fabs(omegaZ_dps) < 0.15f) {
        omegaZ_dps = 0.0f;
    }

    // 3. Numerical Integration: θ_yaw = ∫ ω_z dt
    currentYawDeg += omegaZ_dps * dt;

    // Print telemetry at ~25 Hz (every 40 ms) while integrating at full loop speed
    static unsigned long lastPrintMs = 0;
    if (millis() - lastPrintMs >= 40) {
        lastPrintMs = millis();
        Serial.printf("RawZ: %6d | Rate: %7.2f dps | Yaw Angle: %7.2f deg\n",
                      rawZ, omegaZ_dps, currentYawDeg);
    }
}