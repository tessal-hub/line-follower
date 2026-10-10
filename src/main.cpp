#include <Arduino.h>
#include "config.h"
#include "motors.h"
#include "sensors.h"
#include "encoders.h"
#include "display.h"

enum AppMode {
    MODE_IDLE,
    MODE_RUNNING,
    MODE_LINE_HOLD,
    MODE_MENU
};

static AppMode appMode = MODE_IDLE;

// Flash NVS Configuration
static CarConfig activeConfig;
static CarConfig menuConfig;
static int menuSelectedIndex = 0;
static bool menuIsEditing = false;

// Button States
static bool lastSw1 = HIGH;
static bool lastSw2 = HIGH;
static bool lastSw3 = HIGH;

// Timing & State Variables
static uint32_t lastCtrlUs = 0;
static uint32_t lastDebugPrintMs = 0;
static uint32_t brakeHoldUntilMs = 0;
static uint32_t lineLostStartTime = 0;
static float lastRunError = 0.0f;
static float lastHoldError = 0.0f;
static bool runFirstStep = true;
static bool holdFirstStep = true;
static int8_t lastKnownDir = 1;

void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.flush();

    pinMode(PIN_BTN_SW1, INPUT_PULLUP);
    pinMode(PIN_BTN_SW2, INPUT_PULLUP);
    pinMode(PIN_BTN_SW3, INPUT_PULLUP);

    configLoad(activeConfig);
    motorsSetActiveBrake(activeConfig.activeBrakeEn);

    sensorsInit();
    motorsInit();
    encodersInit();
    displayInit();

    Serial.println("\n=======================================================");
    Serial.println("[ESP32 LINE FOLLOWER READY (Pure Direct PD)]");
    Serial.printf("  Config: Boost:%.0f Base:%.0f Cnr:%.0f Kp:%.1f Kd:%.1f ABS:%s\n",
        activeConfig.boostSpeed, activeConfig.baseSpeed, activeConfig.cornerSpeed,
        activeConfig.kpGain, activeConfig.kdGain, activeConfig.activeBrakeEn ? "ON" : "OFF");
    Serial.println("=======================================================\n");
}

void loop() {
    uint32_t now = millis();

    // --- 1. Pushbutton Polling ---
    bool sw1 = digitalRead(PIN_BTN_SW1);
    bool sw2 = digitalRead(PIN_BTN_SW2);
    bool sw3 = digitalRead(PIN_BTN_SW3);

    // ========================================================
    // MODE: OLED TUNING MENU
    // ========================================================
    if (appMode == MODE_MENU) {
        motorsStop();

        // SW1: UP in navigation / [+] in edit
        if (sw1 == LOW && lastSw1 == HIGH) {
            if (!menuIsEditing) {
                menuSelectedIndex = (menuSelectedIndex + TOTAL_MENU_ITEMS - 1) % TOTAL_MENU_ITEMS;
            } else {
                switch (menuSelectedIndex) {
                    case MENU_ITEM_BOOST_SPD:    menuConfig.boostSpeed  = constrain(menuConfig.boostSpeed + 5.0f, CFG_BOOST_SPD_MIN, CFG_BOOST_SPD_MAX); break;
                    case MENU_ITEM_BASE_SPD:     menuConfig.baseSpeed   = constrain(menuConfig.baseSpeed + 5.0f, CFG_BASE_SPD_MIN, CFG_BASE_SPD_MAX); break;
                    case MENU_ITEM_CORNER_SPD:   menuConfig.cornerSpeed = constrain(menuConfig.cornerSpeed + 5.0f, CFG_CORNER_SPD_MIN, CFG_CORNER_SPD_MAX); break;
                    case MENU_ITEM_KP_GAIN:      menuConfig.kpGain      = constrain(menuConfig.kpGain + 0.5f, CFG_KP_MIN, CFG_KP_MAX); break;
                    case MENU_ITEM_KD_GAIN:      menuConfig.kdGain      = constrain(menuConfig.kdGain + 1.0f, CFG_KD_MIN, CFG_KD_MAX); break;
                    case MENU_ITEM_KC_GAIN:      menuConfig.kCurveGain  = constrain(menuConfig.kCurveGain + 0.5f, CFG_KC_MIN, CFG_KC_MAX); break;
                    case MENU_ITEM_ACTIVE_BRAKE: 
                        menuConfig.activeBrakeEn = !menuConfig.activeBrakeEn;
                        motorsSetActiveBrake(menuConfig.activeBrakeEn);
                        break;
                }
            }
            delay(150);
        }
        lastSw1 = sw1;

        // SW2: DOWN in navigation / [-] in edit
        if (sw2 == LOW && lastSw2 == HIGH) {
            if (!menuIsEditing) {
                menuSelectedIndex = (menuSelectedIndex + 1) % TOTAL_MENU_ITEMS;
            } else {
                switch (menuSelectedIndex) {
                    case MENU_ITEM_BOOST_SPD:    menuConfig.boostSpeed  = constrain(menuConfig.boostSpeed - 5.0f, CFG_BOOST_SPD_MIN, CFG_BOOST_SPD_MAX); break;
                    case MENU_ITEM_BASE_SPD:     menuConfig.baseSpeed   = constrain(menuConfig.baseSpeed - 5.0f, CFG_BASE_SPD_MIN, CFG_BASE_SPD_MAX); break;
                    case MENU_ITEM_CORNER_SPD:   menuConfig.cornerSpeed = constrain(menuConfig.cornerSpeed - 5.0f, CFG_CORNER_SPD_MIN, CFG_CORNER_SPD_MAX); break;
                    case MENU_ITEM_KP_GAIN:      menuConfig.kpGain      = constrain(menuConfig.kpGain - 0.5f, CFG_KP_MIN, CFG_KP_MAX); break;
                    case MENU_ITEM_KD_GAIN:      menuConfig.kdGain      = constrain(menuConfig.kdGain - 1.0f, CFG_KD_MIN, CFG_KD_MAX); break;
                    case MENU_ITEM_KC_GAIN:      menuConfig.kCurveGain  = constrain(menuConfig.kCurveGain - 0.5f, CFG_KC_MIN, CFG_KC_MAX); break;
                    case MENU_ITEM_ACTIVE_BRAKE: 
                        menuConfig.activeBrakeEn = !menuConfig.activeBrakeEn;
                        motorsSetActiveBrake(menuConfig.activeBrakeEn);
                        break;
                }
            }
            delay(150);
        }
        lastSw2 = sw2;

        // SW3: SELECT / TOGGLE EDIT / EXECUTE
        if (sw3 == LOW && lastSw3 == HIGH) {
            if (menuIsEditing) {
                menuIsEditing = false;
            } else {
                if (menuSelectedIndex <= MENU_ITEM_ACTIVE_BRAKE) {
                    menuIsEditing = true;
                } else if (menuSelectedIndex == MENU_ITEM_SAVE_EXIT) {
                    if (menuConfig.baseSpeed < menuConfig.cornerSpeed) menuConfig.baseSpeed = menuConfig.cornerSpeed;
                    if (menuConfig.boostSpeed < menuConfig.baseSpeed) menuConfig.boostSpeed = menuConfig.baseSpeed;
                    configSave(menuConfig);
                    activeConfig = menuConfig;
                    motorsSetActiveBrake(activeConfig.activeBrakeEn);
                    displayShowMessage("SAVED", "CONFIG SAVED TO NVS", 1000);
                    appMode = MODE_IDLE;
                } else if (menuSelectedIndex == MENU_ITEM_CANCEL) {
                    menuConfig = activeConfig;
                    motorsSetActiveBrake(activeConfig.activeBrakeEn);
                    displayShowMessage("CANCEL", "CHANGES DISCARDED", 800);
                    appMode = MODE_IDLE;
                } else if (menuSelectedIndex == MENU_ITEM_RESTORE_DEF) {
                    configResetDefaults(menuConfig);
                    motorsSetActiveBrake(menuConfig.activeBrakeEn);
                    displayShowMessage("RESET", "RESTORED DEFAULTS", 800);
                } else if (menuSelectedIndex == MENU_ITEM_TEST_LINE_HOLD) {
                    appMode = MODE_LINE_HOLD;
                    holdFirstStep = true;
                    motorsStop();
                    displayShowMessage("LINE HOLD", "BENCH TEST MODE", 1000);
                }
            }
            delay(200);
        }
        lastSw3 = sw3;

        MenuDisplayData mData;
        mData.selectedIndex  = menuSelectedIndex;
        mData.isEditing      = menuIsEditing;
        mData.boostSpeed     = menuConfig.boostSpeed;
        mData.baseSpeed      = menuConfig.baseSpeed;
        mData.cornerSpeed    = menuConfig.cornerSpeed;
        mData.kpGain         = menuConfig.kpGain;
        mData.kdGain         = menuConfig.kdGain;
        mData.kCurveGain     = menuConfig.kCurveGain;
        mData.activeBrakeEn  = menuConfig.activeBrakeEn;
        displayUpdateMenu(mData);

        delay(15);
        return;
    }

    // --- Pushbutton Actions ---
    // SW1: Calibrate (Idle) / Emergency Stop (Active)
    if (sw1 == LOW && lastSw1 == HIGH) {
        if (appMode == MODE_IDLE) {
            sensorsCalibrate(10000);
        } else {
            appMode = MODE_IDLE;
            brakeHoldUntilMs = now + 350;
            motorsBrake();
            displayShowMessage("E-STOP", "EMERGENCY STOP!", 1200);
        }
    }
    lastSw1 = sw1;

    // SW2: Tap = RUN/STOP, Hold >= 450ms = LINE-HOLD
    static uint32_t sw2PressStartTime = 0;
    static bool sw2HoldTriggered = false;
    static AppMode sw2ModeOnPress = MODE_IDLE;

    if (sw2 == LOW && lastSw2 == HIGH) {
        sw2ModeOnPress = appMode;
        if (appMode == MODE_RUNNING || appMode == MODE_LINE_HOLD) {
            appMode = MODE_IDLE;
            brakeHoldUntilMs = now + 350;
            motorsBrake();
            displayShowMessage("STOPPED", "CAR IN IDLE", 800);
        } else if (appMode == MODE_IDLE) {
            sw2PressStartTime = now;
            sw2HoldTriggered = false;
        }
    } else if (sw2 == LOW && !sw2HoldTriggered) {
        if (sw2ModeOnPress == MODE_IDLE && now - sw2PressStartTime >= 450) {
            sw2HoldTriggered = true;
            appMode = MODE_LINE_HOLD;
            holdFirstStep = true;
            motorsStop();
            displayShowMessage("LINE HOLD", "BENCH TEST MODE", 1000);
        }
    } else if (sw2 == HIGH && lastSw2 == LOW) {
        if (sw2ModeOnPress == MODE_IDLE && !sw2HoldTriggered) {
            appMode = MODE_RUNNING;
            runFirstStep = true;
            encodersResetTicks();
            brakeHoldUntilMs = 0;
            lineLostStartTime = 0;
            lastCtrlUs = micros();
            displayShowMessage("START", "FULL SPEED RUN", 1000);
        }
    }
    lastSw2 = sw2;

    // SW3: Open Menu (Idle) / Stop (Active)
    if (sw3 == LOW && lastSw3 == HIGH) {
        if (appMode == MODE_IDLE) {
            appMode = MODE_MENU;
            motorsStop();
            menuConfig = activeConfig;
            menuSelectedIndex = 0;
            menuIsEditing = false;
            displayClearMessage();

            MenuDisplayData mData;
            mData.selectedIndex  = 0;
            mData.isEditing      = false;
            mData.boostSpeed     = menuConfig.boostSpeed;
            mData.baseSpeed      = menuConfig.baseSpeed;
            mData.cornerSpeed    = menuConfig.cornerSpeed;
            mData.kpGain         = menuConfig.kpGain;
            mData.kdGain         = menuConfig.kdGain;
            mData.kCurveGain     = menuConfig.kCurveGain;
            mData.activeBrakeEn  = menuConfig.activeBrakeEn;
            displayUpdateMenu(mData);
            return;
        } else if (appMode == MODE_RUNNING || appMode == MODE_LINE_HOLD) {
            appMode = MODE_IDLE;
            brakeHoldUntilMs = now + 350;
            motorsBrake();
            displayShowMessage("IDLE", "CAR IN IDLE", 800);
        }
    }
    lastSw3 = sw3;

    // ========================================================
    // MODE 1: IDLE
    // ========================================================
    if (appMode == MODE_IDLE) {
        if (Serial.available()) {
            char cmd = (char)Serial.read();
            if (cmd == 'h' || cmd == 'H') {
                appMode = MODE_LINE_HOLD;
                holdFirstStep = true;
                motorsStop();
                displayShowMessage("LINE HOLD", "BENCH TEST MODE", 1000);
                return;
            }
        }

        if (now < brakeHoldUntilMs) {
            motorsBrake();
        } else {
            motorsStop();
        }

        if (now - lastDebugPrintMs > 100) {
            lastDebugPrintMs = now;
            bool dummyLine;
            int dummy90;
            float currentErr = sensorsGetError(0.0f, dummyLine, dummy90);

            sensorsPrintDebug(currentErr);
            sensorsPrintRaw();
            int norm[8];
            sensorsGetNormValues(norm);
            displayUpdateSnapshot(
                DISP_MODE_IDLE,
                currentErr,
                dummyLine,
                norm,
                encodersGetLeftSpeedMmS(),
                encodersGetRightSpeedMmS(),
                encodersGetLeftDistanceMm(),
                encodersGetRightDistanceMm(),
                0, 0, 0.0f,
                sensorsIsCalibrated(),
                activeConfig.baseSpeed
            );
        }
        delay(5);
        return;
    }

    // ========================================================
    // MODE 2: LINE-HOLD BENCH TEST (Speed = 0)
    // ========================================================
    if (appMode == MODE_LINE_HOLD) {
        uint32_t nowUs = micros();
        if (nowUs - lastCtrlUs < (CONTROL_LOOP_MS * 1000UL)) return;
        lastCtrlUs = nowUs;

        encodersUpdateSpeed();
        LineDetection det = sensorsProcess();
        int leftPwm = 0;
        int rightPwm = 0;

        if (det.lineDetected) {
            float error = det.positionMm;
            float pTerm = activeConfig.kpGain * error;
            float dTerm = holdFirstStep ? 0.0f : activeConfig.kdGain * (error - lastHoldError);
            holdFirstStep = false;
            lastHoldError = error;

            int steer = (int)constrain(pTerm + dTerm, -200.0f, 200.0f);
            leftPwm  = steer;
            rightPwm = -steer;
            motorsSet(leftPwm, rightPwm);
        } else {
            motorsStop();
            holdFirstStep = true;
        }

        if (now - lastDebugPrintMs > 80) {
            lastDebugPrintMs = now;
            int norm[8];
            sensorsGetNormValues(norm);
            displayUpdateSnapshot(
                DISP_MODE_LINE_HOLD,
                det.positionMm,
                det.lineDetected,
                norm,
                encodersGetLeftSpeedMmS(),
                encodersGetRightSpeedMmS(),
                encodersGetLeftDistanceMm(),
                encodersGetRightDistanceMm(),
                leftPwm,
                rightPwm,
                0.0f,
                sensorsIsCalibrated()
            );
        }
        return;
    }

    // ========================================================
    // MODE 3: RUNNING (250 Hz Pure PD Control)
    // ========================================================
    uint32_t nowUs = micros();
    if (nowUs - lastCtrlUs < (CONTROL_LOOP_MS * 1000UL)) return;
    lastCtrlUs += (CONTROL_LOOP_MS * 1000UL);
    if (nowUs - lastCtrlUs > (CONTROL_LOOP_MS * 2000UL)) lastCtrlUs = nowUs;

    // 1. Encoders & Odometry
    encodersUpdateSpeed();
    float vMeasLMmS = encodersGetLeftSpeedMmS();
    float vMeasRMmS = encodersGetRightSpeedMmS();

    // 2. Đọc cảm biến
    LineDetection det = sensorsProcess();
    float error = 0.0f;

    if (det.lineDetected) {
        error = det.positionMm;
        lineLostStartTime = 0;
        lastKnownDir = (error >= 0.0f) ? 1 : -1;
    } else {
        // Mất line: dừng an toàn sau timeout
        if (lineLostStartTime == 0) lineLostStartTime = now;
        if (now - lineLostStartTime > LINE_LOST_TIMEOUT_MS) {
            appMode = MODE_IDLE;
            brakeHoldUntilMs = now + 350;
            motorsBrake();
            displayShowMessage("SAFETY STOP", "LINE LOST", 1500);
            return;
        }

        // Quét tìm lại vạch theo hướng vừa thoát
        int sweepPwm = (int)constrain(activeConfig.cornerSpeed, 40.0f, 80.0f);
        int leftPwm  = (lastKnownDir > 0) ? sweepPwm : -(sweepPwm / 2);
        int rightPwm = (lastKnownDir > 0) ? -(sweepPwm / 2) : sweepPwm;
        motorsSet(leftPwm, rightPwm);
        runFirstStep = true;
        return;
    }

    // 3. Pure PD Steering
    float deltaErr = runFirstStep ? 0.0f : (error - lastRunError);
    runFirstStep = false;
    lastRunError = error;

    float pTerm = activeConfig.kpGain * error;
    float dTerm = activeConfig.kdGain * deltaErr;
    float steerPwm = pTerm + dTerm;

    // 4. Tốc độ tiến
    float basePwm = activeConfig.baseSpeed;
    if (fabsf(error) > 8.0f) {
        basePwm = activeConfig.cornerSpeed;
    } else if (fabsf(error) < 3.0f) {
        basePwm = activeConfig.boostSpeed;
    }

    // 5. Phân bổ công suất bánh xe
    int leftPwm  = (int)roundf(basePwm + steerPwm);
    int rightPwm = (int)roundf(basePwm - steerPwm);

    leftPwm  = constrain(leftPwm,  -255, 255);
    rightPwm = constrain(rightPwm, -255, 255);
    motorsSet(leftPwm, rightPwm);

    // 6. Telemetry & Display
    if (now - lastDebugPrintMs >= 1000) {
        lastDebugPrintMs = now;
        if (Serial.availableForWrite() >= 96) {
            Serial.printf("[PD RUN] e:%+5.1f mm | Spd:%.0f | L:%+4d R:%+4d\n",
                error, basePwm, leftPwm, rightPwm);
        }
    }

    static uint32_t lastRunDispMs = 0;
    if (now - lastRunDispMs > 80) {
        lastRunDispMs = now;
        int norm[8];
        sensorsGetNormValues(norm);
        displayUpdateSnapshot(
            DISP_MODE_RUNNING,
            error,
            det.lineDetected,
            norm,
            vMeasLMmS,
            vMeasRMmS,
            encodersGetLeftDistanceMm(),
            encodersGetRightDistanceMm(),
            leftPwm,
            rightPwm,
            0.0f,
            sensorsIsCalibrated(),
            activeConfig.baseSpeed
        );
    }
}