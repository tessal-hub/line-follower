#pragma once
#include <Arduino.h>

enum DisplayMode {
    DISP_MODE_IDLE = 0,
    DISP_MODE_RUNNING = 1,
    DISP_MODE_LINE_HOLD = 2,
    DISP_MODE_CALIBRATING = 3,
    DISP_MODE_MENU = 4
};

struct MenuDisplayData {
    int   selectedIndex;
    bool  isEditing;
    float boostSpeedMmS;
    float baseSpeedMmS;
    float cornerSpeedMmS;
    float kpGain;
    float kdGain;
    float kCurveGain;
    bool  activeBrakeEn;
};

void displayInit();

void displayUpdateSnapshot(
    DisplayMode mode,
    float error,
    bool lineDetected,
    const int *sensorNorm,
    float leftSpeedMmS,
    float rightSpeedMmS,
    float leftDistMm,
    float rightDistMm,
    int leftPwm,
    int rightPwm,
    float rate,
    bool isCalibrated,
    float baseSpeedMmS = 1250.0f
);

void displayUpdateMenu(const MenuDisplayData &menu);

void displaySetCalibProgress(uint32_t elapsedMs, uint32_t totalMs);
void displayShowMessage(const char *title, const char *msg, uint32_t durationMs = 1500);
void displayClearMessage();
