#include <Arduino.h>
#include "Config.h"

// ==================== PWM & MOTOR TUNING CONSTANTS ====================
const uint32_t PWM_FREQ_HZ    = 20000;  // 20 kHz ultrasonic frequency
const uint8_t  PWM_RES_BITS   = 8;      // 8-bit PWM range: 0 to 255

// Mechanical gearbox trim (-20 to +20) to correct straight-line drift
int leftMotorTrim  = 0;
int rightMotorTrim = 0;

// ==================== TB6612FNG LOW-LEVEL DRIVER ====================
void initMotors() {
    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_AIN2, OUTPUT);
    pinMode(PIN_BIN1, OUTPUT);
    pinMode(PIN_BIN2, OUTPUT);

    // ESP32 Core 3.x unified LEDC attachment (20 kHz, 8-bit)
    ledcAttach(PIN_PWMA, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttach(PIN_PWMB, PWM_FREQ_HZ, PWM_RES_BITS);

    // Start in Active Short-Brake mode
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, HIGH);
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, HIGH);
    ledcWrite(PIN_PWMA, 0);
    ledcWrite(PIN_PWMB, 0);
}

// Speed range: -255 (Full Reverse) to +255 (Full Forward), 0 = Active Short-Brake
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    if (leftSpeed != 0)  leftSpeed  += (leftSpeed > 0)  ? leftMotorTrim  : -leftMotorTrim;
    if (rightSpeed != 0) rightSpeed += (rightSpeed > 0) ? rightMotorTrim : -rightMotorTrim;

    leftSpeed  = constrain(leftSpeed,  -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    // --- LEFT MOTOR (Channel A) ---
    if (leftSpeed > 0) {
        digitalWrite(PIN_AIN1, HIGH);
        digitalWrite(PIN_AIN2, LOW);
        ledcWrite(PIN_PWMA, leftSpeed);
    } else if (leftSpeed < 0) {
        digitalWrite(PIN_AIN1, LOW);
        digitalWrite(PIN_AIN2, HIGH);
        ledcWrite(PIN_PWMA, -leftSpeed);
    } else {
        digitalWrite(PIN_AIN1, HIGH);
        digitalWrite(PIN_AIN2, HIGH);
        ledcWrite(PIN_PWMA, 0);
    }

    // --- RIGHT MOTOR (Channel B) ---
    if (rightSpeed > 0) {
        digitalWrite(PIN_BIN1, HIGH);
        digitalWrite(PIN_BIN2, LOW);
        ledcWrite(PIN_PWMB, rightSpeed);
    } else if (rightSpeed < 0) {
        digitalWrite(PIN_BIN1, LOW);
        digitalWrite(PIN_BIN2, HIGH);
        ledcWrite(PIN_PWMB, -rightSpeed);
    } else {
        digitalWrite(PIN_BIN1, HIGH);
        digitalWrite(PIN_BIN2, HIGH);
        ledcWrite(PIN_PWMB, 0);
    }
}

void brakeMotors() {
    setMotorSpeeds(0, 0);
}

// ==================== DIAGNOSTIC SEQUENCES ====================
void runDirectionTest() {
    Serial.println("\n--- [TEST 1] Individual Wheel Direction Check ---");
    Serial.println("Spinning LEFT wheel forward (PWM 120)...");
    setMotorSpeeds(120, 0);
    delay(1200);
    brakeMotors();
    delay(500);

    Serial.println("Spinning RIGHT wheel forward (PWM 120)...");
    setMotorSpeeds(0, 120);
    delay(1200);
    brakeMotors();
    delay(500);
}

void runDeadbandSweep() {
    Serial.println("\n--- [TEST 2] Static Friction Deadband Sweep ---");
    Serial.println("Note exact PWM where wheels overcome friction and start rolling:");
    for (int pwm = 20; pwm <= 110; pwm += 5) {
        Serial.printf("Testing PWM = %3d\n", pwm);
        setMotorSpeeds(pwm, pwm);
        delay(350);
    }
    brakeMotors();
    delay(800);
}

void runStraightLineTest(int basePwm, uint32_t durationMs) {
    Serial.printf("\n--- [TEST 3] Open-Loop Straight Run (PWM %d for %lu ms) ---\n", basePwm, durationMs);
    Serial.println("Starting in 1 second... Stand clear!");
    digitalWrite(PIN_LED3, HIGH);
    delay(1000);

    setMotorSpeeds(basePwm, basePwm);
    delay(durationMs);
    brakeMotors();

    digitalWrite(PIN_LED3, LOW);
    Serial.println(">>> [BRAKED] Measure lateral drift from straight line (Target < 3 cm).\n");
}

void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(PIN_LED3, OUTPUT);
    pinMode(PIN_KEY3, INPUT_PULLUP);
    digitalWrite(PIN_LED3, LOW);

    initMotors();

    Serial.println("\n===================================================");
    Serial.println(" [PHASE 05] TB6612FNG 20kHz Open-Loop Motor Drive");
    Serial.println("===================================================");
    Serial.println("SAFETY: Lift bot off ground for Test 1 & 2, or place on 2m floor.");
    Serial.println("Press KEY3 (GPIO 16) to execute motor test sequence...\n");
}

void loop() {
    digitalWrite(PIN_LED3, (millis() / 400) % 2);

    if (digitalRead(PIN_KEY3) == LOW) {
        delay(200);
        runDirectionTest();
        runDeadbandSweep();
        runStraightLineTest(180, 1350);
        Serial.println("Sequence finished! Press KEY3 again to repeat.\n");
    }
}