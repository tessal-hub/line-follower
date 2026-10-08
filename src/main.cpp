#include <Arduino.h>
#include "config.h"
#include "motors.h"
#include "sensors.h"
#include "encoders.h"
#include "display.h"
#include "curvature.h"
#include "speed_controller.h"
#include "track_profile.h"
#include "state_machine.h"

enum AppMode {
    MODE_IDLE,
    MODE_RUNNING,
    MODE_LINE_HOLD,
    MODE_MENU
};

static AppMode appMode = MODE_IDLE;

// Dynamic Flash NVS Configuration
static CarConfig activeConfig;
static CarConfig menuConfig;
static int menuSelectedIndex = 0;
static bool menuIsEditing = false;
static const int TOTAL_MENU_ITEMS = 10;

// Straight-Line Rocket Acceleration Detection
static uint32_t straightTrackStartTime = 0;
static bool inStraightTrack = false;

// Slew-rate limited target speed profile
static float profiledTargetSpeedMmS = 900.0f;
static float lastError = 0.0f;
static uint32_t lastRunUs = 0;
static float filteredRunError = 0.0f;
static float lastRunError = 0.0f;
static float filteredRunRate = 0.0f;

static bool lastSw1 = HIGH;
static bool lastSw2 = HIGH;
static bool lastSw3 = HIGH;
static uint32_t sw2PressStartTime = 0;
static bool sw2LongTriggered = false;

static uint32_t lastDebugPrintMs = 0;
static uint32_t lastControlMs = 0;
static uint32_t lastCtrlUs = 0;
static float runLastValidEMm = 0.0f;
static uint8_t runLostLineCycles = 0;

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
    curvatureInit();
    speedControllerInit();
    trackProfileInit();
    stateMachineInit();

    Serial.println("\n=======================================================");
    Serial.println("[ESP32 HYPER-SPEED LINE FOLLOWER READY (250 Hz Loop)]");
    Serial.println("  Encoders: 4x Quadrature Decoding (275 ticks/rev, ~0.78mm/tick)");
    Serial.println("  OLED 1.3\" (SH1106 / SSD1306): I2C SDA=21 SCL=22 Active");
    Serial.println("  SW1 (GPIO 13): Calibrate (Idle) / Emergency Stop (Active)");
    Serial.println("  SW2 (GPIO 14): Chay duong dua (Vong kin toc do Encoders mm/s)");
    Serial.println("  SW3 (GPIO 27): 1-Click = MENU CAI DAT OLED");
    Serial.printf("  Config: Boost:%.0f Base:%.0f Cnr:%.0f Kp:%.1f Kd:%.1f Kc:%.1f ABS:%s\n",
        activeConfig.boostSpeedMmS, activeConfig.baseSpeedMmS, activeConfig.cornerSpeedMmS,
        activeConfig.kpGain, activeConfig.kdGain, activeConfig.kCurveGain,
        activeConfig.activeBrakeEn ? "ON" : "OFF");
    Serial.println("=======================================================\n");
}

void loop() {
    uint32_t now = millis();

    // --- 1. Pushbutton Polling ---
    bool sw1 = digitalRead(PIN_BTN_SW1);
    bool sw2 = digitalRead(PIN_BTN_SW2);
    bool sw3 = digitalRead(PIN_BTN_SW3);

    // ========================================================
    // MODE: ON-BOARD OLED TUNING MENU
    // ========================================================
    if (appMode == MODE_MENU) {
        motorsStop();

        // SW1: UP in navigation / [+] in edit
        if (sw1 == LOW && lastSw1 == HIGH) {
            if (!menuIsEditing) {
                menuSelectedIndex = (menuSelectedIndex + TOTAL_MENU_ITEMS - 1) % TOTAL_MENU_ITEMS;
            } else {
                switch (menuSelectedIndex) {
                    case 0: menuConfig.boostSpeedMmS  = constrain(menuConfig.boostSpeedMmS + 50.0f, 1000.0f, 2500.0f); break;
                    case 1: menuConfig.baseSpeedMmS   = constrain(menuConfig.baseSpeedMmS + 50.0f, 100.0f, 2200.0f); break;
                    case 2: menuConfig.cornerSpeedMmS = constrain(menuConfig.cornerSpeedMmS + 50.0f, 100.0f, 1500.0f); break;
                    case 3: menuConfig.kpGain         = constrain(menuConfig.kpGain + 1.0f, 0.0f, 80.0f); break;
                    case 4: menuConfig.kdGain         = constrain(menuConfig.kdGain + 1.0f, 0.0f, 80.0f); break;
                    case 5: menuConfig.kCurveGain     = constrain(menuConfig.kCurveGain + 0.5f, 0.0f, 25.0f); break;
                    case 6: 
                        menuConfig.activeBrakeEn  = !menuConfig.activeBrakeEn;
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
                    case 0: menuConfig.boostSpeedMmS  = constrain(menuConfig.boostSpeedMmS - 50.0f, 1000.0f, 2500.0f); break;
                    case 1: menuConfig.baseSpeedMmS   = constrain(menuConfig.baseSpeedMmS - 50.0f, 100.0f, 2200.0f); break;
                    case 2: menuConfig.cornerSpeedMmS = constrain(menuConfig.cornerSpeedMmS - 50.0f, 100.0f, 1500.0f); break;
                    case 3: menuConfig.kpGain         = constrain(menuConfig.kpGain - 1.0f, 0.0f, 80.0f); break;
                    case 4: menuConfig.kdGain         = constrain(menuConfig.kdGain - 1.0f, 0.0f, 80.0f); break;
                    case 5: menuConfig.kCurveGain     = constrain(menuConfig.kCurveGain - 0.5f, 0.0f, 25.0f); break;
                    case 6: 
                        menuConfig.activeBrakeEn  = !menuConfig.activeBrakeEn;
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
                if (menuSelectedIndex <= 6) {
                    menuIsEditing = true;
                } else if (menuSelectedIndex == 7) { // [ SAVE & EXIT ]
                    if (menuConfig.baseSpeedMmS < menuConfig.cornerSpeedMmS) {
                        menuConfig.baseSpeedMmS = menuConfig.cornerSpeedMmS;
                    }
                    if (menuConfig.boostSpeedMmS < menuConfig.baseSpeedMmS) {
                        menuConfig.boostSpeedMmS = menuConfig.baseSpeedMmS;
                    }
                    configSave(menuConfig);
                    activeConfig = menuConfig;
                    motorsSetActiveBrake(activeConfig.activeBrakeEn);
                    displayShowMessage("SAVED", "CONFIG SAVED TO NVS", 1000);
                    appMode = MODE_IDLE;
                } else if (menuSelectedIndex == 8) { // [ CANCEL ]
                    menuConfig = activeConfig;
                    motorsSetActiveBrake(activeConfig.activeBrakeEn);
                    displayShowMessage("CANCEL", "CHANGES DISCARDED", 800);
                    appMode = MODE_IDLE;
                } else if (menuSelectedIndex == 9) { // [ RESTORE DEF ]
                    configResetDefaults(menuConfig);
                    motorsSetActiveBrake(menuConfig.activeBrakeEn);
                    displayShowMessage("RESET", "RESTORED DEFAULTS", 800);
                }
            }
            delay(200);
        }
        lastSw3 = sw3;

        MenuDisplayData mData;
        mData.selectedIndex  = menuSelectedIndex;
        mData.isEditing      = menuIsEditing;
        mData.boostSpeedMmS  = menuConfig.boostSpeedMmS;
        mData.baseSpeedMmS   = menuConfig.baseSpeedMmS;
        mData.cornerSpeedMmS = menuConfig.cornerSpeedMmS;
        mData.kpGain         = menuConfig.kpGain;
        mData.kdGain         = menuConfig.kdGain;
        mData.kCurveGain     = menuConfig.kCurveGain;
        mData.activeBrakeEn  = menuConfig.activeBrakeEn;
        displayUpdateMenu(mData);

        delay(15);
        return;
    }

    // --- Pushbutton Actions in Operational Modes ---
    // SW1: Calibrate (Idle) / Emergency Stop (Active modes)
    if (sw1 == LOW && lastSw1 == HIGH) {
        if (appMode == MODE_IDLE) {
            sensorsCalibrate(10000);
        } else {
            appMode = MODE_IDLE;
            speedControllerReset();
            motorsBrake();
            displayShowMessage("E-STOP", "EMERGENCY STOP!", 1200);
            Serial.println("[E-STOP] Emergency stop triggered by SW1!");
        }
        delay(250);
    }
    lastSw1 = sw1;

    // SW2: Click = RUN/STOP | Giữ >=800ms = GIỮ LINE TẠI CHỖ (LINE-HOLD BENCH TEST)
    if (sw2 == LOW && lastSw2 == HIGH) {
        sw2PressStartTime = now;
        sw2LongTriggered = false;
    } else if (sw2 == LOW && !sw2LongTriggered) {
        if (now - sw2PressStartTime >= 800) {
            sw2LongTriggered = true;
            if (appMode == MODE_LINE_HOLD) {
                appMode = MODE_IDLE;
                motorsStop();
                displayShowMessage("IDLE", "BENCH TEST EXIT", 800);
                Serial.println("[SW2] Thoat che do GIU LINE TAI CHO ve IDLE.");
            } else {
                appMode = MODE_LINE_HOLD;
                motorsStop();
                speedControllerReset();
                displayShowMessage("LINE HOLD", "BENCH TEST MODE", 1000);
                Serial.println("\n[SW2 LONG PRESS] -> Vao che do GIU LINE TAI CHO (Day xe nhe nhang de test).");
            }
        }
    } else if (sw2 == HIGH && lastSw2 == LOW) {
        if (!sw2LongTriggered) {
            // Nhan nha nhanh (Short click): Toggle RUNNING / IDLE
            if (appMode == MODE_RUNNING) {
                appMode = MODE_IDLE;
                motorsStop();
                speedControllerReset();
                displayShowMessage("STOPPED", "CAR IN IDLE", 1000);
                Serial.printf("[STATE] Stopped. Dist L: %.1f mm | R: %.1f mm | Speed L: %.0f mm/s | R: %.0f mm/s\n",
                    encodersGetLeftDistanceMm(), encodersGetRightDistanceMm(),
                    encodersGetLeftSpeedMmS(), encodersGetRightSpeedMmS());
            } else if (appMode == MODE_LINE_HOLD) {
                appMode = MODE_IDLE;
                motorsStop();
                displayShowMessage("IDLE", "CAR IN IDLE", 800);
            } else {
                appMode = MODE_RUNNING;
                curvatureReset();
                stateMachineReset();
                encodersResetTicks();
                speedControllerReset();
                profiledTargetSpeedMmS = activeConfig.cornerSpeedMmS; // Khoi dong muot ma tu corner speed
                straightTrackStartTime = 0;
                inStraightTrack = false;
                lastError = 0.0f;
                filteredRunError = 0.0f;
                lastRunError = 0.0f;
                filteredRunRate = 0.0f;
                runLostLineCycles = 0;
                runLastValidEMm = 0.0f;
                lastControlMs = now;
                lastCtrlUs = micros();
                lastRunUs = micros();
                displayShowMessage("START", "FULL SPEED RUN", 1000);
                Serial.printf("[STATE] RUNNING started! Base:%.0f Cnr:%.0f Kp:%.1f Kd:%.1f Kc:%.1f ABS:%s (250 Hz Loop)\n",
                    activeConfig.baseSpeedMmS, activeConfig.cornerSpeedMmS,
                    activeConfig.kpGain, activeConfig.kdGain, activeConfig.kCurveGain,
                    activeConfig.activeBrakeEn ? "ON" : "OFF");
            }
        }
    }
    lastSw2 = sw2;

    // SW3: 1-Click = MỞ MENU CÀI ĐẶT OLED TỨC THÌ
    if (sw3 == LOW && lastSw3 == HIGH) {
        appMode = MODE_MENU;
        motorsStop();
        speedControllerReset();
        stateMachineReset();
        menuConfig = activeConfig;
        menuSelectedIndex = 0;
        menuIsEditing = false;
        displayClearMessage();

        MenuDisplayData mData;
        mData.selectedIndex  = 0;
        mData.isEditing      = false;
        mData.boostSpeedMmS  = menuConfig.boostSpeedMmS;
        mData.baseSpeedMmS   = menuConfig.baseSpeedMmS;
        mData.cornerSpeedMmS = menuConfig.cornerSpeedMmS;
        mData.kpGain         = menuConfig.kpGain;
        mData.kdGain         = menuConfig.kdGain;
        mData.kCurveGain     = menuConfig.kCurveGain;
        mData.activeBrakeEn  = menuConfig.activeBrakeEn;
        displayUpdateMenu(mData);

        Serial.println("\n>>> [MENU] Menu cai dat OLED mo qua SW3 <<<");
        lastSw3 = LOW;
        delay(250);
        return;
    }
    lastSw3 = sw3;

    // ========================================================
    // MODE 1: IDLE - Show Live Sensor ASCII Bar + Encoders
    // ========================================================
    if (appMode == MODE_IDLE) {
        motorsStop();
        if (now - lastDebugPrintMs > 100) {
            lastDebugPrintMs = now;
            bool dummyLine;
            int dummy90;
            float currentErr = sensorsGetError(0.0f, dummyLine, dummy90);

            sensorsPrintDebug(currentErr);
            sensorsPrintRaw();
            Serial.printf("  Encoders -> L: %ld (%5.1fmm) | R: %ld (%5.1fmm)\n",
                encodersGetLeftTicks(), encodersGetLeftDistanceMm(),
                encodersGetRightTicks(), encodersGetRightDistanceMm());

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
                activeConfig.baseSpeedMmS
            );
        }
        delay(5);
        return;
    }

    // ========================================================
    // MODE: LINE-HOLD BENCH TEST (Zero Forward Speed)
    // ========================================================
    if (appMode == MODE_LINE_HOLD) {
        uint32_t nowUsHold = micros();
        if (nowUsHold - lastCtrlUs < 4000) {
            return;
        }
        float dtHold = (lastCtrlUs > 0) ? (float)(nowUsHold - lastCtrlUs) * 1e-6f : 0.004f;
        if (dtHold <= 0.0005f || dtHold > 0.05f) dtHold = 0.004f;
        lastCtrlUs = nowUsHold;
        lastControlMs = now;

        encodersUpdateSpeed();

        LineDetection det = sensorsProcess();
        float eMm = det.positionMm;

        static float filteredHoldError = 0.0f;
        static float lastHoldError = 0.0f;
        static float filteredHoldRate = 0.0f;

        int leftPwm = 0;
        int rightPwm = 0;

        if (!det.lineDetected) {
            // Khi nhac xe khoi vach: Dung dong co an toan, khong quay giat
            motorsStop();
            leftPwm  = 0;
            rightPwm = 0;
            filteredHoldRate = 0.0f;
            lastHoldError = 0.0f;
            filteredHoldError = 0.0f;
        } else {
            // Loc thong thap nhe sai so de triet tieu nhieu quang
            filteredHoldError = 0.50f * eMm + 0.50f * filteredHoldError;
            float rawRate = (filteredHoldError - lastHoldError) / dtHold;
            lastHoldError = filteredHoldError;
            filteredHoldRate = 0.20f * rawRate + 0.80f * filteredHoldRate;

            float absErr = fabsf(filteredHoldError);
            if (absErr < 1.0f) {
                // Vung tam [-1.0mm, +1.0mm]: Tha long em ai, zero vibration
                motorsStop();
                leftPwm  = 0;
                rightPwm = 0;
            } else {
                float sign = (filteredHoldError > 0.0f) ? 1.0f : -1.0f;
                // Vach lech phai (e > 0) -> Banh trai tien (+), banh phai lui (-) de xoay dau xe sang phai
                float excess = absErr - 1.0f;
                float pTorque = sign * (25.0f + excess * activeConfig.kpGain * 0.45f + excess * excess * 0.12f);
                float dTorque = filteredHoldRate * activeConfig.kdGain * 0.005f;

                int steerPwm = (int)constrain(pTorque + dTorque, -200.0f, 200.0f);
                leftPwm  = steerPwm;
                rightPwm = -steerPwm;
                motorsSet(leftPwm, rightPwm);
            }
        }

        // Cap nhat man hinh OLED va Serial
        if (now - lastDebugPrintMs > 80) {
            lastDebugPrintMs = now;
            int norm[8];
            sensorsGetNormValues(norm);
            displayUpdateSnapshot(
                DISP_MODE_LINE_HOLD,
                eMm,
                det.lineDetected,
                norm,
                encodersGetLeftSpeedMmS(),
                encodersGetRightSpeedMmS(),
                encodersGetLeftDistanceMm(),
                encodersGetRightDistanceMm(),
                leftPwm,
                rightPwm,
                filteredHoldRate,
                sensorsIsCalibrated()
            );

            if (Serial.availableForWrite() >= 64) {
                Serial.printf("[HOLD] e:%+5.1f mm | PWM L:%+4d R:%+4d | Spd L:%.0f R:%.0f | %s\n",
                    eMm, leftPwm, rightPwm, encodersGetLeftSpeedMmS(), encodersGetRightSpeedMmS(),
                    det.lineDetected ? "TRACKING" : "LINE LOST");
            }
        }
        return;
    }

    // ========================================================
    // MODE 2: FULL RUNNING TRACK MODE (250 Hz Deterministic Loop)
    // ========================================================
    uint32_t nowUs = micros();
    if (nowUs - lastCtrlUs < 4000) {
        return;
    }
    if (nowUs - lastCtrlUs > 16000) {
        lastCtrlUs = nowUs;
    } else {
        lastCtrlUs += 4000;
    }
    lastControlMs = now;

    // 1. Cập nhật đo đạc Encoder (4x Quadrature Decoding) & Odometry
    encodersUpdateSpeed();
    float vMeasLMmS   = encodersGetLeftSpeedMmS();
    float vMeasRMmS   = encodersGetRightSpeedMmS();
    float vMeasAvgMmS = (vMeasLMmS + vMeasRMmS) * 0.5f;
    float distMm      = (encodersGetLeftDistanceMm() + encodersGetRightDistanceMm()) * 0.5f;
    float distM       = distMm * 0.001f;

    // 2. Thu thập và xử lý Cảm biến quang
    LineDetection det = sensorsProcess();

    float eMm = 0.0f;
    if (det.lineDetected) {
        eMm = det.positionMm;
        runLastValidEMm = eMm;
        runLostLineCycles = 0;
    } else {
        runLostLineCycles++;
        // LỌC CHỐNG CHỚP LÓA BỀ MẶT BÓNG (3 chu kỳ đầu = 12ms):
        // Giữ vị trí hợp lệ gần nhất, mượt mà nội suy sang hướng ghi nhớ
        if (runLostLineCycles <= 3) {
            eMm = runLastValidEMm;
        } else {
            // Vạch trượt ra ngoài thanh cảm biến trong góc cua gắt:
            int dir = (sensorsGetLastGoodDir() != 0) ? sensorsGetLastGoodDir() : sm.lastValidDir;
            float targetExtMm = (dir > 0) ? 42.0f : -42.0f;
            eMm = 0.70f * runLastValidEMm + 0.30f * targetExtMm;
            runLastValidEMm = eMm;
        }
    }
    float eM  = eMm * 0.001f;

    // Chu kỳ dt thực tế
    float dt = (lastRunUs > 0) ? (float)(nowUs - lastRunUs) * 1e-6f : 0.004f;
    if (dt <= 0.0005f || dt > 0.05f) dt = 0.004f;
    lastRunUs = nowUs;

    // 3. Máy trạng thái 3 State (RUN, SHARP_TURN, RECOVERY) kèm Debounce 32ms
    static float dErrorFiltMmS = 0.0f;
    StateMachineContext sm = stateMachineUpdate(
        det.lineDetected,
        eMm,
        dErrorFiltMmS,
        det.confidence,
        det.isExtrapolated,
        det.turn90,
        vMeasAvgMmS,
        now
    );

    // Xử lý mất line timeout
    if (sm.emergencyStopTriggered) {
        appMode = MODE_IDLE;
        motorsBrake();
        displayShowMessage("SAFETY STOP", "LINE LOST TIMEOUT", 2000);
        Serial.println("[SAFETY STOP] Line lost timeout exceeded! Motors shut down.");
        return;
    }

    // 4. Xác định Base PWM từ tốc độ cài đặt (Cho động cơ JGB37-520 12V 1590RPM)
    int baseCruisingPwm = constrain((int)roundf(activeConfig.baseSpeedMmS * 0.100f + 25.0f), 70, 230);
    int cornerPwm       = constrain((int)roundf(activeConfig.cornerSpeedMmS * 0.090f + 35.0f), 55, 140);
    int boostPwm        = constrain((int)roundf(activeConfig.boostSpeedMmS * 0.105f + 35.0f), 80, 255);

    // 5. Tính toán đạo hàm sai số:
    filteredRunError = 0.45f * eMm + 0.55f * filteredRunError;
    float rawRate = (filteredRunError - lastRunError) / dt;
    lastRunError = filteredRunError;
    filteredRunRate = 0.20f * rawRate + 0.80f * filteredRunRate;

    float absErr = fabsf(filteredRunError);
    int basePwm = baseCruisingPwm;

    // Tốc độ tiến tối thiểu trong cua gắt / ôm góc:
    // Tuyệt đối không cho basePwm = 0 để tránh xe đứng yên xoay compa tại chỗ!
    int minCornerPwm = (int)roundf(cornerPwm * 0.65f); // ~52 PWM

    if (!det.lineDetected && runLostLineCycles > 3) {
        // Line tạm thời vượt ra ngoài mép trong góc cua gắt:
        // Duy trì tốc độ bò cua tiến đều để xe vẽ vòng cung quét bắt lại line!
        basePwm = minCornerPwm;
    } else {
        // Nhận diện đường thẳng có Hysteresis: Bám tâm <= 2.2mm trong 100ms là BUNG BOOST!
        if (absErr <= 2.2f && det.lineDetected) {
            if (!inStraightTrack) {
                inStraightTrack = true;
                straightTrackStartTime = now;
            }
            if (now - straightTrackStartTime > 100) {
                basePwm = boostPwm; // BUNG TỐC ĐỘ ROCKET TRÊN ĐOẠN THẲNG!
            }
        } else if (absErr > 3.8f) {
            inStraightTrack = false;
        }

        if (!inStraightTrack || absErr > 2.2f) {
            if (absErr <= 8.0f) {
                // Thoát thẳng vào cua êm dịu: Hạ tốc độ từ baseCruisingPwm xuống cornerPwm
                float t = constrain((absErr - 2.0f) / 6.0f, 0.0f, 1.0f);
                basePwm = (int)roundf(baseCruisingPwm - (baseCruisingPwm - cornerPwm) * t);
            } else {
                // Cua gắt / góc nhọn (|e| > 8.0mm): Duy trì tốc độ bò góc minCornerPwm để xe luôn khỏe, không bị nghẽn
                float t = constrain((absErr - 8.0f) / 8.0f, 0.0f, 1.0f);
                basePwm = (int)roundf(cornerPwm - (cornerPwm - minCornerPwm) * t);
            }
        }
    }

    // 7. Tính toán Steer PWM: ĐIỀU KHIỂN LIÊN TỤC C0 (TRIỆT TIÊU 100% RUNG LẮC & XOAY COMPA)
    float steerPwm = 0.0f;

    if (!det.lineDetected && runLostLineCycles > 3) {
        // Quét góc cua gắt: Đánh lái mạnh theo hướng vạch thoát ra ngoài
        int dir = (sensorsGetLastGoodDir() != 0) ? sensorsGetLastGoodDir() : sm.lastValidDir;
        steerPwm = (dir > 0) ? 145.0f : -145.0f;
    } else {
        // BÁM LINE CHỦ ĐỘNG LIÊN TỤC VÀ MƯỢT MÀ (ZERO-SHOCK, ZERO-LIMIT-CYCLE):
        float sign = (filteredRunError > 0.0f) ? 1.0f : -1.0f;

        // Vùng tâm êm ái:
        // Hàm liên tục C0 bắt đầu từ 0 PWM khi e = 0.6mm, TUYỆT ĐỐI KHÔNG CÓ BƯỚC NHẢY ĐỘT NGỘT!
        float pTorque = 0.0f;
        if (absErr > 0.6f) {
            float excess = absErr - 0.6f;
            // Khâu P: Tuyến tính nhẹ ở đầu + Phi tuyến theo Kc khi vào góc sâu
            float curveFactor = activeConfig.kCurveGain * 0.016f; // Kc=16 -> 0.256f
            pTorque = sign * (excess * (activeConfig.kpGain * 0.45f) + excess * excess * curveFactor);
        }

        // Khâu D (Derivative damping): Giảm chấn quán tính xoay liên tục khắp dải sai số,
        // Dập tắt rung lắc và hãm phanh khi xe đang phóng về tâm line (triệt tiêu 100% overshoot!)
        float dTorque = filteredRunRate * (activeConfig.kdGain * 0.0032f);

        steerPwm = pTorque + dTorque;
        steerPwm = constrain(steerPwm, -215.0f, 215.0f);
    }

    // 8. Phân bổ vi sai công suất bánh xe:
    int leftPwm  = (int)roundf(basePwm + steerPwm);
    int rightPwm = (int)roundf(basePwm - steerPwm);

    // Bảo toàn chênh lệch bẻ lái khi một bánh chạm trần 255
    int over = max(leftPwm, rightPwm) - 255;
    if (over > 0) {
        leftPwm  -= over;
        rightPwm -= over;
    }

    // Giới hạn dải công suất [-220, 255] (cho phép bánh trong quay lùi tới -220 để bẻ gập góc nhọn < 90 độ)
    leftPwm  = constrain(leftPwm,  -220, 255);
    rightPwm = constrain(rightPwm, -220, 255);

    // 9. Xuất trực tiếp công suất ra DRV8871
    motorsSet(leftPwm, rightPwm);

    static uint32_t lastRunDispMs = 0;
    if (now - lastRunDispMs > 80) {
        lastRunDispMs = now;
        int norm[8];
        sensorsGetNormValues(norm);
        displayUpdateSnapshot(
            DISP_MODE_RUNNING,
            eMm,
            det.lineDetected,
            norm,
            vMeasLMmS,
            vMeasRMmS,
            encodersGetLeftDistanceMm(),
            encodersGetRightDistanceMm(),
            leftPwm,
            rightPwm,
            filteredRunRate,
            sensorsIsCalibrated(),
            activeConfig.baseSpeedMmS
        );
    }
}