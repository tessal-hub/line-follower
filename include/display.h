#pragma once
#include <Arduino.h>

enum DisplayMode {
    DISP_MODE_IDLE = 0,
    DISP_MODE_RUNNING = 1,
    DISP_MODE_LINE_HOLD = 2,
    DISP_MODE_CALIBRATING = 3,
    DISP_MODE_MENU = 4
};

#define TOTAL_MENU_ITEMS 11

enum MenuItemIndex {
    MENU_ITEM_BOOST_SPD     = 0,
    MENU_ITEM_BASE_SPD      = 1,
    MENU_ITEM_CORNER_SPD    = 2,
    MENU_ITEM_KP_GAIN       = 3,
    MENU_ITEM_KD_GAIN       = 4,
    MENU_ITEM_KC_GAIN       = 5,
    MENU_ITEM_ACTIVE_BRAKE  = 6,
    MENU_ITEM_SAVE_EXIT     = 7,
    MENU_ITEM_CANCEL        = 8,
    MENU_ITEM_RESTORE_DEF   = 9,
    MENU_ITEM_TEST_LINE_HOLD = 10
};

struct MenuDisplayData {
    int   selectedIndex;
    bool  isEditing;
    float boostSpeed;
    float baseSpeed;
    float cornerSpeed;
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
    float baseSpeed = 140.0f
);

void displayUpdateMenu(const MenuDisplayData &menu);
void displaySetCalibProgress(uint32_t elapsedMs, uint32_t totalMs);
void displayShowMessage(const char *title, const char *msg, uint32_t durationMs = 1500);
void displayClearMessage();
