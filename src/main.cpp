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

enum TrackState {
    STATE_NORMAL_PID,
    STATE_PIVOT_90_RIGHT,
    STATE_PIVOT_90_LEFT
};

static AppMode appMode = MODE_IDLE;
static TrackState trackState = STATE_NORMAL_PID;

// Dynamic Flash NVS Configuration (Upgrade 2)
static CarConfig activeConfig;
static CarConfig menuConfig;
static int menuSelectedIndex = 0;
static bool menuIsEditing = false;
static const int TOTAL_MENU_ITEMS = 10;

// Button timing for SW3 Long Press detection (Hold >=1s -> Menu)
static uint32_t sw3PressStartTime = 0;
static bool sw3LongTriggered = false;

// Straight-Line Rocket Acceleration Profile (Hyper-Cruising)
static uint32_t straightTrackStartTime = 0;
static float currentCruisingSpeedMmS = 1250.0f;
static bool inStraightTrack = false;

static float lastError = 0.0f;
static float lastLineError = 0.0f;
static float dErrorFilt = 0.0f;
static bool wasLineLost = false;
static uint32_t lastRunUs = 0;

static inline void pidResetState(float currentErr = 0.0f) {
    lastError = currentErr;
    lastLineError = currentErr;
    dErrorFilt = 0.0f;
    lastRunUs = micros();
}
static uint32_t pivotStartTime = 0;
static uint32_t postCornerUntilMs = 0;
static uint32_t lineLostStartTime = 0;

static bool lastSw1 = HIGH;
static bool lastSw2 = HIGH;
static bool lastSw3 = HIGH;

static uint32_t lastDebugPrintMs = 0;
static uint32_t lastControlMs = 0;
static uint32_t lastCtrlUs = 0;

void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.flush();

    pinMode(PIN_BTN_SW1, INPUT_PULLUP);
    pinMode(PIN_BTN_SW2, INPUT_PULLUP);
    pinMode(PIN_BTN_SW3, INPUT_PULLUP);

    configLoad(activeConfig);
    currentCruisingSpeedMmS = activeConfig.baseSpeedMmS;
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
    Serial.println("  OLED 1.3\" (SH1106 / SSD1306): I2C SDA=21 SCL=22 Active");
    Serial.println("  SW1 (GPIO 13): Calibrate (Idle) / Emergency Stop (Active)");
    Serial.println("  SW2 (GPIO 14): Chạy đường đua (Vòng kín tốc độ Encoders mm/s)");
    Serial.println("  SW3 (GPIO 27): Chạm nhanh = LINE-HOLD | Giữ >=1s = MENU CÀI ĐẶT OLED");
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
    // MODE 4: ON-BOARD OLED TUNING MENU (Upgrade 2)
    // ========================================================
    if (appMode == MODE_MENU) {
        motorsStop();

        // SW1: UP in navigation / [+] in edit
        if (sw1 == LOW && lastSw1 == HIGH) {
            if (!menuIsEditing) {
                menuSelectedIndex = (menuSelectedIndex + TOTAL_MENU_ITEMS - 1) % TOTAL_MENU_ITEMS;
            } else {
                switch (menuSelectedIndex) {
                    case 0: menuConfig.boostSpeedMmS  = constrain(menuConfig.boostSpeedMmS + 50.0f, 0.0f, 2500.0f); break;
                    case 1: menuConfig.baseSpeedMmS   = constrain(menuConfig.baseSpeedMmS + 50.0f, 100.0f, 2200.0f); break;
                    case 2: menuConfig.cornerSpeedMmS = constrain(menuConfig.cornerSpeedMmS + 50.0f, 100.0f, 1500.0f); break;
                    case 3: menuConfig.kpGain         = constrain(menuConfig.kpGain + 1.0f, 0.0f, 80.0f); break;
                    case 4: menuConfig.kdGain         = constrain(menuConfig.kdGain + 1.0f, 0.0f, 80.0f); break;
                    case 5: menuConfig.kCurveGain     = constrain(menuConfig.kCurveGain + 0.5f, 0.0f, 25.0f); break;
                    case 6: menuConfig.activeBrakeEn  = !menuConfig.activeBrakeEn; break;
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
                    case 0: menuConfig.boostSpeedMmS  = constrain(menuConfig.boostSpeedMmS - 50.0f, 0.0f, 2500.0f); break;
                    case 1: menuConfig.baseSpeedMmS   = constrain(menuConfig.baseSpeedMmS - 50.0f, 100.0f, 2200.0f); break;
                    case 2: menuConfig.cornerSpeedMmS = constrain(menuConfig.cornerSpeedMmS - 50.0f, 100.0f, 1500.0f); break;
                    case 3: menuConfig.kpGain         = constrain(menuConfig.kpGain - 1.0f, 0.0f, 80.0f); break;
                    case 4: menuConfig.kdGain         = constrain(menuConfig.kdGain - 1.0f, 0.0f, 80.0f); break;
                    case 5: menuConfig.kCurveGain     = constrain(menuConfig.kCurveGain - 0.5f, 0.0f, 25.0f); break;
                    case 6: menuConfig.activeBrakeEn  = !menuConfig.activeBrakeEn; break;
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
                    currentCruisingSpeedMmS = activeConfig.baseSpeedMmS;
                    displayShowMessage("SAVED", "CONFIG SAVED TO NVS", 1000);
                    appMode = MODE_IDLE;
                } else if (menuSelectedIndex == 8) { // [ CANCEL ]
                    menuConfig = activeConfig;
                    displayShowMessage("CANCEL", "CHANGES DISCARDED", 800);
                    appMode = MODE_IDLE;
                } else if (menuSelectedIndex == 9) { // [ RESTORE DEF ]
                    configResetDefaults(menuConfig);
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
            trackState = STATE_NORMAL_PID;
            speedControllerReset();
            motorsBrake();
            displayShowMessage("E-STOP", "EMERGENCY STOP!", 1200);
            Serial.println("[E-STOP] Emergency stop triggered by SW1!");
        }
        delay(250);
    }
    lastSw1 = sw1;

    // SW2: Toggle RUNNING track mode
    if (sw2 == LOW && lastSw2 == HIGH) {
        if (appMode == MODE_RUNNING) {
            appMode = MODE_IDLE;
            motorsStop();
            speedControllerReset();
            trackState = STATE_NORMAL_PID;
            lineLostStartTime = 0;
            displayShowMessage("STOPPED", "CAR IN IDLE", 1000);
            Serial.printf("[STATE] Stopped. Dist L: %.1f mm | R: %.1f mm | Speed L: %.0f mm/s | R: %.0f mm/s\n",
                encodersGetLeftDistanceMm(), encodersGetRightDistanceMm(),
                encodersGetLeftSpeedMmS(), encodersGetRightSpeedMmS());
        } else {
            appMode = MODE_RUNNING;
            curvatureReset();
            stateMachineReset();
            lastError = 0.0f;
            lastControlMs = now;
            lastCtrlUs = micros();
            trackState = STATE_NORMAL_PID;
            postCornerUntilMs = 0;
            lineLostStartTime = 0;
            encodersResetTicks();
            speedControllerReset();
            currentCruisingSpeedMmS = activeConfig.baseSpeedMmS;
            straightTrackStartTime = 0;
            inStraightTrack = false;
            pidResetState(0.0f);
            displayShowMessage("START", "FULL SPEED RUN", 1000);
            Serial.printf("[STATE] RUNNING started! Base:%.0f Cnr:%.0f Kp:%.1f Kd:%.1f Kc:%.1f ABS:%s (250 Hz Loop)\n",
                activeConfig.baseSpeedMmS, activeConfig.cornerSpeedMmS,
                activeConfig.kpGain, activeConfig.kdGain, activeConfig.kCurveGain,
                activeConfig.activeBrakeEn ? "ON" : "OFF");
        }
        delay(250);
    }
    lastSw2 = sw2;

    // SW3: 1-Click = MỞ MENU CÀI ĐẶT OLED TỨC THÌ (Tuyệt đối không bị kẹt STOP)
    if (sw3 == LOW && lastSw3 == HIGH) {
        appMode = MODE_MENU;
        motorsStop();
        speedControllerReset();
        stateMachineReset();
        menuConfig = activeConfig;
        menuSelectedIndex = 0;
        menuIsEditing = false;
        displayClearMessage(); // Hủy bỏ ngay lập tức bất kỳ popup STOP nào

        // Cập nhật ngay dữ liệu Menu lên màn hình
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

        Serial.println("\n>>> [MENU] Menu cài đặt OLED mở tức thì qua SW3 <<<");
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
    // MODE 2: LINE-HOLD BENCH TEST (Zero Forward Speed)
    // ========================================================
    if (appMode == MODE_LINE_HOLD) {
        if (now - lastControlMs < CONTROL_LOOP_MS) {
            return;
        }
        uint32_t holdDtMs = now - lastControlMs;
        lastControlMs = now;
        float dt = (holdDtMs > 0 && holdDtMs < 50) ? (holdDtMs * 0.001f) : 0.001f;

        bool lineDetected = false;
        int dummy90 = 0;
        float error = sensorsGetError(lastError, lineDetected, dummy90);

        static float filteredHoldError = 0.0f;
        static float lastHoldError = 0.0f;
        static float filteredHoldRate = 0.0f;

        int leftPwm = 0;
        int rightPwm = 0;

        if (!lineDetected) {
            // Line lost on bench: gracefully coast to stop (never whip violently at 135 PWM!)
            motorsStop();
            leftPwm  = 0;
            rightPwm = 0;
            filteredHoldRate = 0.0f;
            lastHoldError = error;
            filteredHoldError = error;
            lastError = error;
        } else {
            // Smooth 1-pole EMA filter on error: eliminates optical sensor noise spikes
            filteredHoldError = 0.40f * error + 0.60f * filteredHoldError;

            float rawRate = (filteredHoldError - lastHoldError) / dt;
            lastHoldError = filteredHoldError;
            lastError = error;
            filteredHoldRate = 0.08f * rawRate + 0.92f * filteredHoldRate;

            float absErr = fabsf(filteredHoldError);
            if (absErr < 0.08f) {
                // Centered deadband: 100% silent coast, zero torque, zero vibration
                motorsStop();
                leftPwm  = 0;
                rightPwm = 0;
            } else {
                float sign = (filteredHoldError > 0.0f) ? 1.0f : -1.0f;
                // Purely continuous restoring torque starting smoothly from 0.0 PWM:
                // No cliffs, no 40 PWM jumps!
                float excess = absErr - 0.08f;
                float pTorque = sign * constrain(excess * 105.0f + excess * excess * 35.0f, 0.0f, 150.0f);
                float dTorque = constrain(filteredHoldRate * 3.5f, -40.0f, 40.0f);

                int pwr = (int)constrain(pTorque + dTorque, -155.0f, 155.0f);
                leftPwm  = pwr;
                rightPwm = -pwr;
                motorsSet(leftPwm, rightPwm);
            }
        }

        // Live telemetry output every 80ms
        if (now - lastDebugPrintMs > 80) {
            lastDebugPrintMs = now;
            int norm[8];
            sensorsGetNormValues(norm);
            displayUpdateSnapshot(
                DISP_MODE_LINE_HOLD,
                error,
                lineDetected,
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

            if (!lineDetected) {
                Serial.printf("[HOLD-RECOVER] Line LOST! Sweeping %s (Dir:%+d) | L:%+4d R:%+4d\n",
                    (sensorsGetLastGoodDir() > 0 ? "RIGHT" : "LEFT"), sensorsGetLastGoodDir(), leftPwm, rightPwm);
            } else {
                Serial.printf("[HOLD] Err:%+5.2f Rate:%+5.1f | L:%+4d R:%+4d | ",
                    error, filteredHoldRate, leftPwm, rightPwm);
                sensorsPrintDebug(error);
            }
        }
        return;
    }

    // ========================================================
    // MODE 3: FULL RUNNING TRACK MODE (250 Hz Loop)
    // ========================================================
    // ========================================================
    // MODE 3: FULL RUNNING TRACK MODE (250 Hz Deterministic Loop)
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

    // 1. Cập nhật đo đạc Encoder & Odometry
    encodersUpdateSpeed();
    float vMeasLMmS   = encodersGetLeftSpeedMmS();
    float vMeasRMmS   = encodersGetRightSpeedMmS();
    float vMeasAvgMmS = (vMeasLMmS + vMeasRMmS) * 0.5f;
    float distMm      = (encodersGetLeftDistanceMm() + encodersGetRightDistanceMm()) * 0.5f;
    float distM       = distMm * 0.001f;

    // 2. Thu thập và xử lý Cảm biến
    LineDetection det = sensorsProcess();
    float eMm = det.lineDetected ? det.positionMm : (sensorsGetLastGoodDir() > 0 ? 45.0f : -45.0f);
    float eM  = eMm * 0.001f;

    // Chu kỳ dt thực tế
    float dt = (lastRunUs > 0) ? (float)(nowUs - lastRunUs) * 1e-6f : 0.004f;
    if (dt <= 0.0005f || dt > 0.05f) dt = 0.004f;
    lastRunUs = nowUs;

    // 3. Máy trạng thái 3 State (RUN, SHARP_TURN, RECOVERY) kèm Hysteresis
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

    // 4. Xác định Tốc độ Đích v_target [mm/s]
    float vTargetLinearMmS = activeConfig.baseSpeedMmS;

    if (sm.currentState == STATE_RECOVERY) {
        vTargetLinearMmS = 350.0f; // Tốc độ vừa đủ để bẻ góc tìm vạch
        // Nếu mắt đã bắt đầu chạm vào vạch, dùng NGAY vị trí thực tế eMm thay vì mù quáng xoay 45mm!
        if (det.lineDetected) {
            eM = det.positionMm * 0.001f;
        } else {
            eM = (sm.lastValidDir > 0) ? 0.045f : -0.045f;
        }
    } else if (sm.currentState == STATE_SHARP_TURN) {
        vTargetLinearMmS = activeConfig.cornerSpeedMmS;
    } else { // STATE_RUN
        // Kiểm tra Speed Profile theo quãng đường s [m]
        float vProfile = trackProfileGetTargetSpeed(distM);
        if (vProfile > 0.0f) {
            vTargetLinearMmS = vProfile;
        } else {
            // Fallback phản ứng theo độ lệch vị trí & de/dt
            vTargetLinearMmS = trackProfileComputeReactiveFallback(
                eMm,
                dErrorFiltMmS,
                det.confidence,
                activeConfig.baseSpeedMmS,
                activeConfig.cornerSpeedMmS,
                vMeasAvgMmS
            );
        }

        // Safety Override: nếu lệch quá lớn hoặc mất confidence thì ghìm tốc ngay
        if (trackProfileCheckSafetyOverride(eMm, det.confidence)) {
            vTargetLinearMmS = activeConfig.cornerSpeedMmS;
        }
    }

    // 5. Curvature Controller: Pure Pursuit -> kappa_cmd [m^-1] -> omega [rad/s] -> v_left, v_right [mm/s]
    float kpCurv = activeConfig.kpGain;           // [m^-2] (OLED Kp 10 - 50)
    float kdCurv = activeConfig.kdGain * 0.025f;  // [s/m^2] (OLED Kd 10 - 70 quy đổi ra s/m^2)
    float kcCurv = activeConfig.kCurveGain;       // [m^-2] Thành phần phi tuyến bù lực lái khi vào góc
    CurvatureOutput curOut = curvatureCompute(
        eM,
        dt,
        vTargetLinearMmS,
        kpCurv,
        kdCurv,
        kcCurv,
        activeConfig.boostSpeedMmS,
        (sm.currentState == STATE_SHARP_TURN)
    );
    dErrorFiltMmS = curOut.dErrorMs * 1000.0f;

    // Ghi nhận điểm map nếu đang trong lap thăm dò
    trackProfileRecordStep(distM, curOut.kappaCmd, det.confidence);

    // 6. Speed Controller: Feedforward từng bánh + PI chung cho vận tốc tịnh tiến
    float aTargetMmS2 = (curOut.targetSpeedMmS - vMeasAvgMmS) / dt;
    SpeedControllerOutput spdOut = speedControllerCompute(
        curOut.targetSpeedMmS,
        curOut.vLeftMmS,
        curOut.vRightMmS,
        aTargetMmS2,
        vMeasLMmS,
        vMeasRMmS,
        dt,
        11100.0f, // [GIẢ ĐỊNH] Vbat thực tế ~ 11.1V
        11100.0f, // Vbat danh định 11.1V
        (sm.currentState != STATE_RUN) // Freeze integral trong cua gắt / recovery
    );

    // 7. Xuất tín hiệu động cơ qua DRV8871
    motorsSet(spdOut.pwmL, spdOut.pwmR);


    static uint32_t lastLoopUs = 0;
    static uint32_t maxLoopUs = 0;
    static uint32_t lastLoopStatMs = 0;
    uint32_t loopNowUs = micros();
    if (lastLoopUs > 0) {
        uint32_t loopDelta = loopNowUs - lastLoopUs;
        if (loopDelta > maxLoopUs) maxLoopUs = loopDelta;
    }
    lastLoopUs = loopNowUs;

    if (now - lastLoopStatMs >= 1000) {
        lastLoopStatMs = now;
        if (Serial.availableForWrite() >= 96) {
            Serial.printf("[PERF] LoopMax:%u us | e:%+.1f mm | kap:%+.3f | spdL:%.0f spdR:%.0f | pwmL:%d pwmR:%d\n",
                maxLoopUs, eMm, curOut.kappaCmd, vMeasLMmS, vMeasRMmS, spdOut.pwmL, spdOut.pwmR);
        }
        maxLoopUs = 0;
    }

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
            spdOut.pwmL,
            spdOut.pwmR,
            dErrorFiltMmS,
            sensorsIsCalibrated(),
            curOut.targetSpeedMmS
        );
    }
}