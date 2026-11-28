#include <Arduino.h>
#include "Config.h"

// Volatile variables modified inside Hardware Interrupts (ISRs)
volatile long leftTicks  = 0;
volatile long rightTicks = 0;

// 4x Quadrature State Lookup Table
// Index = (prev_state << 2) | curr_state
const int8_t QEM_TABLE[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

volatile uint8_t prevLeftState  = 0;
volatile uint8_t prevRightState = 0;

// ==================== INTERRUPT SERVICE ROUTINES (IRAM_ATTR) ====================
void IRAM_ATTR isrLeftEncoder() {
    uint8_t a = digitalRead(PIN_ENC_L_A);
    uint8_t b = digitalRead(PIN_ENC_L_B);
    uint8_t currState = (a << 1) | b;
    leftTicks = leftTicks + QEM_TABLE[(prevLeftState << 2) | currState];
    prevLeftState = currState;
}

void IRAM_ATTR isrRightEncoder() {
    uint8_t a = digitalRead(PIN_ENC_R_A);
    uint8_t b = digitalRead(PIN_ENC_R_B);
    uint8_t currState = (a << 1) | b;
    rightTicks = rightTicks + QEM_TABLE[(prevRightState << 2) | currState];
    prevRightState = currState;
}

// ==================== SAFE ATOMIC READ & RESET ====================
void getEncoderTicks(long &lTicks, long &rTicks) {
    noInterrupts();
    lTicks = leftTicks;
    rTicks = rightTicks;
    interrupts();
}

void resetEncoders() {
    noInterrupts();
    leftTicks  = 0;
    rightTicks = 0;
    interrupts();
}

void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(PIN_LED3, OUTPUT);
    pinMode(PIN_KEY3, INPUT_PULLUP);
    digitalWrite(PIN_LED3, LOW);

    // GPIO 34, 35, 36, 39 are Input-Only (No internal pull-ups on ESP32 silicon)
    pinMode(PIN_ENC_L_A, INPUT);
    pinMode(PIN_ENC_L_B, INPUT);
    pinMode(PIN_ENC_R_A, INPUT);
    pinMode(PIN_ENC_R_B, INPUT);

    // Initialize initial quadrature states before enabling interrupts
    prevLeftState  = (digitalRead(PIN_ENC_L_A) << 1) | digitalRead(PIN_ENC_L_B);
    prevRightState = (digitalRead(PIN_ENC_R_A) << 1) | digitalRead(PIN_ENC_R_B);

    // Attach 4x Quadrature Interrupts on BOTH Phase A and Phase B (CHANGE edge)
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrLeftEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_B), isrLeftEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrRightEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_B), isrRightEncoder, CHANGE);

    Serial.println("\n===================================================");
    Serial.println(" [PHASE 03] Wheel Encoders & 4x Quadrature Odometry");
    Serial.println("===================================================");
    Serial.printf("Wheel Diameter : %.1f mm\n", WHEEL_DIAMETER_MM);
    Serial.printf("Scale Factor   : %.4f ticks/mm\n", TICKS_PER_MM);
    Serial.println("Instructions   : Roll wheels forward (ticks must INCREASE).");
    Serial.println("                 Press KEY3 (GPIO 16) to zero counters.\n");
}

void loop() {
    // Zero odometry when KEY3 is pressed
    if (digitalRead(PIN_KEY3) == LOW) {
        resetEncoders();
        digitalWrite(PIN_LED3, HIGH);
        Serial.println("\n>>> [RESET] Odometry Zeroed! Push bot 500.0 mm along ruler...\n");
        delay(250);
        digitalWrite(PIN_LED3, LOW);
    }

    long lTicks = 0, rTicks = 0;
    getEncoderTicks(lTicks, rTicks);

    // Convert ticks to millimeter distance
    float leftDistMM  = (float)lTicks / TICKS_PER_MM;
    float rightDistMM = (float)rTicks / TICKS_PER_MM;
    float avgDistMM   = (leftDistMM + rightDistMM) / 2.0f;

    Serial.printf("L_Ticks: %6ld (%6.1f mm) | R_Ticks: %6ld (%6.1f mm) | Avg: %6.1f mm\n",
                  lTicks, leftDistMM, rTicks, rightDistMM, avgDistMM);

    delay(50); // 20 Hz telemetry stream
}