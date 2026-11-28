#include <Arduino.h>
#include "Config.h"
#include "Motors.h"
#include "Encoders.h"
#include "IMU.h"
#include "LineSensor.h"
#include "MazeSolver.h"
#include "HybridComms.h"

// ==================== RUNTIME TUNING & RUN MODE ====================
enum RunMode {
    MODE_EXPLORE,   // Pass 1: LSRB Maze Mapping + Path Reduction + NVS Save
    MODE_SPRINT     // Pass 2: Pre-computed Shortest Path at 1.5 - 1.83 m/s
};

enum BotState {
    STATE_IDLE,
    STATE_TRACKING,
    STATE_VERIFY_BRANCH,
    STATE_VERIFY_ENDZONE,
    STATE_VERIFY_DEADEND,
    STATE_ALIGN_AXLE,
    STATE_TURNING,
    STATE_FINISHED
};

RunMode  activeMode   = MODE_EXPLORE;
BotState currentState = STATE_IDLE;

float Kp        = DEFAULT_KP;
float Ki        = DEFAULT_KI;
float Kd        = DEFAULT_KD;
int   basePwm   = EXPLORE_BASE_PWM;
int   steerSign = 1;
bool  isArmed   = false;

// State Machine Tracking Variables
long  snapshotTicks   = 0;
bool  seenLeftWing    = false;
bool  seenRightWing   = false;
char  pendingTurnDir  = 'S';
float targetTurnDeg   = 0.0f;

float prevErr         = 0.0f;
float integralErr     = 0.0f;
float currentErr      = 0.0f;
float currentSteer    = 0.0f;

// ==================== TURN & DECISION HELPERS ====================
void startGyroPivot(char dir) {
    Motors::stop();
    IMU::resetTurnYaw();

    if (dir == 'L')      targetTurnDeg =  90.0f;
    else if (dir == 'R') targetTurnDeg = -90.0f;
    else                 targetTurnDeg = 180.0f;

    currentState = STATE_TURNING;
}

void handleJunctionDecision(bool hasLeft, bool hasStraight, bool hasRight, bool isDeadEnd) {
    char chosen = 'S';

    if (activeMode == MODE_EXPLORE) {
        if (isDeadEnd)        chosen = 'B';
        else if (hasLeft)     chosen = 'L';
        else if (hasStraight) chosen = 'S';
        else if (hasRight)    chosen = 'R';
        else                  chosen = 'B';

        // Record moves only at multi-branch junctions or dead-ends
        if (isDeadEnd || ((int)hasLeft + (int)hasStraight + (int)hasRight) >= 2) {
            MazeSolver::recordMove(chosen);
            char msg[96];
            snprintf(msg, sizeof(msg), "[EXPLORE] Move '%c' | Raw: %s\n", chosen, MazeSolver::rawPath);
            HybridComms::log(msg);
        }
    } else {
        // SPRINT MODE: Execute pre-optimized shortest path at multi-branch junctions
        if (((int)hasLeft + (int)hasStraight + (int)hasRight) >= 2) {
            chosen = MazeSolver::nextSprintMove();
            char msg[96];
            snprintf(msg, sizeof(msg), "[SPRINT] Step %d/%d -> '%c'\n",
                     MazeSolver::sprintIdx, MazeSolver::optLen, chosen);
            HybridComms::log(msg);
        } else {
            // Forced single-way 90-degree corner
            chosen = hasLeft ? 'L' : (hasRight ? 'R' : 'S');
        }
    }

    pendingTurnDir = chosen;

    if (chosen == 'S') {
        prevErr = 0.0f;
        currentState = STATE_TRACKING;
        return;
    }

    snapshotTicks = Encoders::getAverageTicks();
    currentState  = STATE_ALIGN_AXLE;
}

void armRun(RunMode mode) {
    activeMode  = mode;
    prevErr     = 0.0f;
    integralErr = 0.0f;

    if (mode == MODE_EXPLORE) {
        MazeSolver::clearPaths();
        Kp      = DEFAULT_KP;
        Kd      = DEFAULT_KD;
        basePwm = EXPLORE_BASE_PWM;
        HybridComms::log("\n[ARMED] PASS 1: LSRB EXPLORATION RUN STARTED!\n");
    } else {
        if (MazeSolver::optLen == 0) {
            HybridComms::log("[WARN] No optimized path in memory! Run Explore first or load NVS.\n");
            return;
        }
        MazeSolver::sprintIdx = 0;
        Kp      = SPRINT_KP;
        Kd      = SPRINT_KD;
        basePwm = SPRINT_CRUISE_PWM;
        char msg[128];
        snprintf(msg, sizeof(msg), "\n[ARMED] PASS 2: 1.8 m/s SPRINT RUN! Path (%d): %s\n",
                 MazeSolver::optLen, MazeSolver::optPath);
        HybridComms::log(msg);
    }

    isArmed      = true;
    currentState = STATE_TRACKING;
    digitalWrite(PIN_LED3, HIGH);
}

void disarmBot(const char* reason) {
    isArmed      = false;
    currentState = STATE_IDLE;
    Motors::stop();
    digitalWrite(PIN_LED3, LOW);
    HybridComms::log(reason);
}

// ==================== CLI COMMAND PARSER (TELNET / SERIAL) ====================
void onCommandReceived(String cmd) {
    cmd.trim();
    cmd.toLowerCase();
    if (cmd.length() == 0) return;

    char reply[192];
    if (cmd.startsWith("set kp "))        { Kp = cmd.substring(7).toFloat(); snprintf(reply, sizeof(reply), "[OK] Kp=%.5f\n", Kp); }
    else if (cmd.startsWith("set ki "))   { Ki = cmd.substring(7).toFloat(); snprintf(reply, sizeof(reply), "[OK] Ki=%.5f\n", Ki); }
    else if (cmd.startsWith("set kd "))   { Kd = cmd.substring(7).toFloat(); snprintf(reply, sizeof(reply), "[OK] Kd=%.5f\n", Kd); }
    else if (cmd.startsWith("set base ")) { basePwm = constrain(cmd.substring(9).toInt(), 0, 255); snprintf(reply, sizeof(reply), "[OK] Base=%d\n", basePwm); }
    else if (cmd == "invert")             { steerSign = -steerSign; snprintf(reply, sizeof(reply), "[OK] SteerSign=%d\n", steerSign); }
    else if (cmd == "zeroyaw")            { IMU::totalYawDeg = 0.0f; snprintf(reply, sizeof(reply), "[OK] Yaw=0.0\n"); }
    else if (cmd == "cal") {
        disarmBot("[CAL] Sweep ARC-16 across white line & black floor for 4s...\n");
        int span = LineSensor::calibrateSweep(4000);
        MazeSolver::saveToNVS(LineSensor::calMin, LineSensor::calMax);
        snprintf(reply, sizeof(reply), "[CAL] Done & Saved to NVS! Min Span = %d\n", span);
    }
    else if (cmd == "start" || cmd == "explore") { armRun(MODE_EXPLORE); return; }
    else if (cmd == "sprint")                    { armRun(MODE_SPRINT);  return; }
    else if (cmd == "stop")                      { disarmBot("[HALT] EMERGENCY STOP!\n"); return; }
    else if (cmd == "load") {
        bool ok = MazeSolver::loadFromNVS(LineSensor::calMin, LineSensor::calMax);
        snprintf(reply, sizeof(reply), ok ? "[NVS] Loaded Path: %s\n" : "[NVS] No saved path found.\n", MazeSolver::optPath);
    }
    else if (cmd == "status") {
        snprintf(reply, sizeof(reply), "[STATUS] Mode:%s | State:%d | Raw:%s | Opt:%s | Base:%d\n",
                 (activeMode == MODE_EXPLORE) ? "EXPLORE" : "SPRINT",
                 currentState,
                 MazeSolver::rawLen ? MazeSolver::rawPath : "-",
                 MazeSolver::optLen ? MazeSolver::optPath : "-",
                 basePwm);
    }
    else {
        snprintf(reply, sizeof(reply), "[ERR] Cmds: cal, explore, sprint, stop, load, invert, zeroyaw, set kp/ki/kd/base <v>, status\n");
    }
    HybridComms::log(reply);
}

// ==================== SETUP ====================
void setup() {
    Serial.begin(115200);
    delay(300);

    pinMode(PIN_LED3, OUTPUT);
    pinMode(PIN_KEY3, INPUT_PULLUP);
    digitalWrite(PIN_LED3, LOW);

    Motors::init();
    Encoders::init();
    LineSensor::init();

    if (!IMU::init()) {
        Serial.println("[FATAL] LSM6DSO IMU not detected on SDA(21)/SCL(22)!");
        while (true) { digitalWrite(PIN_LED3, !digitalRead(PIN_LED3)); delay(120); }
    }
    IMU::calibrateBias(600);

    // Load previously saved calibration & shortest path from NVS Flash (if available)
    bool hasSavedPath = MazeSolver::loadFromNVS(LineSensor::calMin, LineSensor::calMax);

    HybridComms::init(true);

    Serial.println("\n=========================================================");
    Serial.println(" [PHASE 10] PROJECTX AUTONOMOUS MAZE SOLVER READY");
    Serial.println("=========================================================");
    if (hasSavedPath) {
        Serial.printf("[NVS RESTORED] Shortest Path Ready (%d moves): %s\n",
                      MazeSolver::optLen, MazeSolver::optPath);
        Serial.println("KEY3 Controls: 1-Click = SPRINT | Double-Click = EXPLORE | Hold 1.5s = CAL\n");
    } else {
        Serial.println("[NVS EMPTY] Ready for Pass 1 Exploration.");
        Serial.println("KEY3 Controls: 1-Click = EXPLORE | Hold 1.5s = CAL\n");
    }
}

// ==================== STANDALONE KEY3 HANDLER ====================
void handlePhysicalButton() {
    if (digitalRead(PIN_KEY3) == HIGH) return;

    unsigned long t0 = millis();
    while (digitalRead(PIN_KEY3) == LOW) {
        if (millis() - t0 >= 1500) {
            onCommandReceived("cal");
            while (digitalRead(PIN_KEY3) == LOW) delay(10);
            return;
        }
        delay(10);
    }
    if (millis() - t0 < 40) return; // Ignore switch bounce

    // If currently running, single press acts as instant Emergency Stop
    if (isArmed) {
        disarmBot("[KEY3] Emergency Stop!\n");
        return;
    }

    // Check for second click within 320 ms (Double-click forces new Explore run)
    bool doubleClick = false;
    unsigned long waitStart = millis();
    while (millis() - waitStart < 320) {
        if (digitalRead(PIN_KEY3) == LOW) {
            doubleClick = true;
            while (digitalRead(PIN_KEY3) == LOW) delay(10);
            break;
        }
        delay(5);
    }

    if (doubleClick) {
        armRun(MODE_EXPLORE);
    } else {
        // Single click: Auto-select Sprint if optimized path exists, else Explore
        if (MazeSolver::optLen > 0) armRun(MODE_SPRINT);
        else                        armRun(MODE_EXPLORE);
    }
}

// ==================== MAIN 500 Hz DETERMINISTIC LOOP ====================
void loop() {
    HybridComms::poll();
    handlePhysicalButton();

    // 500 Hz (2000 us) Control Loop
    static unsigned long lastLoopUs = 0;
    unsigned long nowUs = micros();
    if (nowUs - lastLoopUs >= 2000) {
        lastLoopUs = nowUs;

        IMU::update();
        ZoneSignature z = LineSensor::readZones();
        currentErr = z.lineErr;

        if (!isArmed) {
            Motors::stop();
            // Slow heartbeat blink when idle (double-pulse if Sprint path is loaded)
            if (MazeSolver::optLen > 0) {
                uint32_t m = millis() % 1200;
                digitalWrite(PIN_LED3, (m < 100 || (m > 220 && m < 320)) ? HIGH : LOW);
            } else {
                digitalWrite(PIN_LED3, (millis() / 600) % 2);
            }
        } else {
            switch (currentState) {
                case STATE_TRACKING: {
                    integralErr = constrain(integralErr + currentErr, -10000.0f, 10000.0f);
                    float derivative = currentErr - prevErr;
                    prevErr = currentErr;

                    currentSteer = steerSign * ((Kp * currentErr) + (Ki * integralErr) + (Kd * derivative));

                    // Dynamic Straight-Corridor Speed Boost during Sprint Mode
                    int effectivePwm = basePwm;
                    if (activeMode == MODE_SPRINT && fabs(currentErr) < 600.0f && fabs(IMU::liveRateZ) < 15.0f) {
                        effectivePwm = MAX_SPRINT_PWM;
                    }

                    Motors::setSpeeds(effectivePwm + currentSteer, effectivePwm - currentSteer);

                    if (z.activeCount >= 14) {
                        snapshotTicks = Encoders::getAverageTicks();
                        seenLeftWing  = true;
                        seenRightWing = true;
                        currentState  = STATE_VERIFY_ENDZONE;
                    } else if (z.leftWing || z.rightWing) {
                        snapshotTicks = Encoders::getAverageTicks();
                        seenLeftWing  = z.leftWing;
                        seenRightWing = z.rightWing;
                        currentState  = STATE_VERIFY_BRANCH;
                    } else if (z.activeCount == 0) {
                        snapshotTicks = Encoders::getAverageTicks();
                        currentState  = STATE_VERIFY_DEADEND;
                    }
                    break;
                }

                case STATE_VERIFY_BRANCH: {
                    if (z.leftWing)  seenLeftWing  = true;
                    if (z.rightWing) seenRightWing = true;

                    float steer = (z.activeCount >= 12) ? 0.0f : (steerSign * Kp * currentErr * 0.35f);
                    int verifyPwm = min(basePwm, 130); // Slow slightly for crisp junction read
                    Motors::setSpeeds(verifyPwm + steer, verifyPwm - steer);

                    if (Encoders::getDistanceDeltaMm(snapshotTicks) >= INCH_FORWARD_MM) {
                        if (z.activeCount >= 14) {
                            currentState = STATE_VERIFY_ENDZONE;
                        } else {
                            handleJunctionDecision(seenLeftWing, z.center, seenRightWing, false);
                        }
                    }
                    break;
                }

                case STATE_VERIFY_ENDZONE: {
                    Motors::setSpeeds(min(basePwm, 130), min(basePwm, 130));
                    float dist = Encoders::getDistanceDeltaMm(snapshotTicks);

                    if (z.activeCount < 12) {
                        handleJunctionDecision(true, z.center, true, false);
                    } else if (dist >= END_ZONE_MM) {
                        Motors::stop();
                        isArmed      = false;
                        currentState = STATE_FINISHED;
                        digitalWrite(PIN_LED3, LOW);

                        if (activeMode == MODE_EXPLORE) {
                            MazeSolver::optimizePath();
                            MazeSolver::saveToNVS(LineSensor::calMin, LineSensor::calMax);

                            char msg[256];
                            snprintf(msg, sizeof(msg),
                                     "\n[SOLVED] GOAL REACHED!\n"
                                     "  Raw Path (%d) : %s\n"
                                     "  Opt Path (%d) : %s  [SAVED TO NVS FLASH]\n"
                                     "  Place at START & press KEY3 (or send 'sprint') for 1.8 m/s Run!\n\n",
                                     MazeSolver::rawLen, MazeSolver::rawPath,
                                     MazeSolver::optLen, MazeSolver::optPath);
                            HybridComms::log(msg);
                        } else {
                            HybridComms::log("\n[VICTORY] SPRINT RUN COMPLETED AT END-ZONE!\n\n");
                        }
                    }
                    break;
                }

                case STATE_VERIFY_DEADEND: {
                    Motors::setSpeeds(100, 100);
                    float dist = Encoders::getDistanceDeltaMm(snapshotTicks);
                    if (z.activeCount > 0) {
                        currentState = STATE_TRACKING;
                    } else if (dist >= DEAD_END_MM) {
                        handleJunctionDecision(false, false, false, true);
                    }
                    break;
                }

                case STATE_ALIGN_AXLE: {
                    float remMm = max(0.0f, WHEEL_CENTER_OFFSET_MM - INCH_FORWARD_MM);
                    Motors::setSpeeds(105, 105);
                    if (Encoders::getDistanceDeltaMm(snapshotTicks) >= remMm) {
                        startGyroPivot(pendingTurnDir);
                    }
                    break;
                }

                case STATE_TURNING: {
                    float remDeg = targetTurnDeg - IMU::turnYawDeg;
                    float absRem = fabs(remDeg);

                    // High-speed snap turn with proportional deceleration & static-friction floor
                    int maxTurnPwm = (activeMode == MODE_SPRINT) ? 185 : 150;
                    int turnPwm    = constrain((int)(absRem * 2.4f + 68.0f), 75, maxTurnPwm);
                    int dirSign    = (remDeg >= 0) ? 1 : -1;

                    Motors::setSpeeds(-dirSign * turnPwm, dirSign * turnPwm);

                    bool lineCaught = (absRem < 18.0f) && z.center;
                    if (lineCaught || absRem <= 2.5f) {
                        Motors::stop();
                        prevErr      = 0.0f;
                        integralErr  = 0.0f;
                        currentState = STATE_TRACKING;
                    }
                    break;
                }

                default:
                    Motors::stop();
                    break;
            }
        }
    }

    // 20 Hz Hybrid UDP Telemetry Stream (Feeds cockpit.py)
    static unsigned long lastTelMs = 0;
    if (millis() - lastTelMs >= 50) {
        lastTelMs = millis();
        const char* activePathStr = (activeMode == MODE_SPRINT && MazeSolver::optLen > 0)
                                    ? MazeSolver::optPath
                                    : MazeSolver::rawPath;

        HybridComms::sendTelemetry(currentErr, currentSteer,
                                   Motors::lastPwmL, Motors::lastPwmR,
                                   IMU::totalYawDeg, IMU::liveRateZ,
                                   LineSensor::normValues, activePathStr);
    }
}