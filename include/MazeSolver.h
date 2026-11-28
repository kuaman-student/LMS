#pragma once
#include <Arduino.h>
#include <Preferences.h>

namespace MazeSolver {
    constexpr int MAX_PATH_LEN = 256;

    Preferences prefs;
    char rawPath[MAX_PATH_LEN];
    int  rawLen = 0;

    char optPath[MAX_PATH_LEN];
    int  optLen = 0;
    int  sprintIdx = 0;

    inline void clearPaths() {
        rawLen = 0;
        optLen = 0;
        sprintIdx = 0;
        rawPath[0] = '\0';
        optPath[0] = '\0';
    }

    inline void recordMove(char dir) {
        if (rawLen < MAX_PATH_LEN - 1) {
            rawPath[rawLen++] = dir;
            rawPath[rawLen]   = '\0';
        }
    }

    // Single-pass stack reduction of dead-end ('B') triplets
    inline void optimizePath() {
        char stk[MAX_PATH_LEN];
        int top = 0;

        for (int i = 0; i < rawLen; i++) {
            stk[top++] = rawPath[i];

            while (top >= 3 && stk[top - 2] == 'B') {
                char a = stk[top - 3];
                char c = stk[top - 1];
                char replacement = '\0';

                if      (a == 'L' && c == 'R') replacement = 'B';
                else if (a == 'L' && c == 'S') replacement = 'R';
                else if (a == 'R' && c == 'L') replacement = 'B';
                else if (a == 'S' && c == 'L') replacement = 'R';
                else if (a == 'S' && c == 'S') replacement = 'B';
                else if (a == 'L' && c == 'L') replacement = 'S';

                if (replacement != '\0') {
                    top -= 3;
                    stk[top++] = replacement;
                } else {
                    break;
                }
            }
        }

        memcpy(optPath, stk, top);
        optPath[top] = '\0';
        optLen = top;
        sprintIdx = 0;
    }

    // Persist optimized path & calibration bounds to non-volatile NVS Flash
    inline void saveToNVS(const int* calMin, const int* calMax) {
        prefs.begin("projectx", false);
        prefs.putString("opt_path", optPath);
        prefs.putBytes("cal_min", calMin, 16 * sizeof(int));
        prefs.putBytes("cal_max", calMax, 16 * sizeof(int));
        prefs.end();
    }

    inline bool loadFromNVS(int* calMin, int* calMax) {
        prefs.begin("projectx", true);
        String saved = prefs.getString("opt_path", "");
        if (prefs.isKey("cal_min") && prefs.isKey("cal_max")) {
            prefs.getBytes("cal_min", calMin, 16 * sizeof(int));
            prefs.getBytes("cal_max", calMax, 16 * sizeof(int));
        }
        prefs.end();

        if (saved.length() > 0 && saved.length() < MAX_PATH_LEN) {
            strncpy(optPath, saved.c_str(), MAX_PATH_LEN - 1);
            optPath[MAX_PATH_LEN - 1] = '\0';
            optLen = saved.length();
            sprintIdx = 0;
            return true;
        }
        return false;
    }

    inline char nextSprintMove() {
        if (sprintIdx < optLen) {
            return optPath[sprintIdx++];
        }
        return 'S'; // Fallback straight if path exhausted before end-zone
    }
}