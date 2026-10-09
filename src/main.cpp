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

static uint32_t lastDebugPrintMs = 0;
static uint32_t lastControlMs = 0;
static uint32_t lastCtrlUs = 0;
static float runLastValidEMm = 0.0f;
static uint8_t runLostLineCycles = 0;
static float vRamp = 0.0f;
static uint32_t brakeHoldUntilMs = 0;

// =========================================================================
// MOTOR DYNAMICS CHARACTERIZATION (Steady-State Sweep & Linear Regression)
// Quét 5 điểm PWM: 60, 100, 140, 180, 220 -> Hồi quy v = (PWM - ks) / kv
// Đo trực tiếp trên sàn (hoặc kê bánh), kiểm tra chiều Encoder polarity
// =========================================================================
static void runMotorCharacterization() {
    Serial.println("\n=======================================================");
    Serial.println("[CHARACTERIZATION] QUET DONG HOC MOTOR & HOI QUY (ks, kv)");
    Serial.println("  Hay dat xe tren san phang (xe se chay tien tung nhat ngan).");
    Serial.println("  He thong se chay 5 muc PWM: 60, 100, 140, 180, 220.");
    Serial.println("  CHU Y: Bam nut SW1 (GPIO 13) bat cu luc nao de E-STOP!");
    Serial.println("=======================================================");
    displayShowMessage("CHAR MOTORS", "SWEEP 5 POINTS", 1500);
    delay(1500);

    const int NUM_POINTS = 5;
    const int TEST_PWM[NUM_POINTS] = {60, 100, 140, 180, 220};
    float measSpeedL[NUM_POINTS] = {0};
    float measSpeedR[NUM_POINTS] = {0};

    motorsStop();
    delay(400);

    // Kiem tra chieu quay Encoder ban dau tai PWM 60
    encodersResetTicks();
    motorsSet(60, 60);
    delay(300);
    long initialL = encodersGetLeftTicks();
    long initialR = encodersGetRightTicks();
    motorsBrake();
    delay(200);

    Serial.printf("[CHAR CHECK] Huong Encoder -> Left: %+ld ticks | Right: %+ld ticks\n", initialL, initialR);
    if (initialL <= 0 || initialR <= 0) {
        Serial.println("\n*******************************************************");
        Serial.println("[ENCODER ALARM] CANH BAO: ENCODER BI NGUOC CHIEU (AM) KHI CHAY TIEN!");
        if (initialL <= 0) Serial.println("  -> Banh TRAI: ticks <= 0! Sua INVERT_LEFT_ENCODER = true!");
        if (initialR <= 0) Serial.println("  -> Banh PHAI: ticks <= 0! Sua INVERT_RIGHT_ENCODER = true!");
        Serial.println("  -> Dung ngay de tranh vong PI bi hoi tiep duong gay mat lai!");
        Serial.println("*******************************************************\n");
        displayShowMessage("ENC ERROR!", "TICKS NEGATIVE", 2500);
        motorsStop();
        return;
    }

    // Quet tung diem PWM
    for (int i = 0; i < NUM_POINTS; i++) {
        int pwm = TEST_PWM[i];
        char stepBuf[24];
        snprintf(stepBuf, sizeof(stepBuf), "PWM %d (Pt %d/5)", pwm, i + 1);
        displayShowMessage("TESTING...", stepBuf, 400);

        Serial.printf("\n[CHAR Pt %d/5] Tang toc PWM = %d ...\n", i + 1, pwm);
        encodersResetTicks();

        // 1. Giai doan tang toc transient (bo qua 400ms dau)
        motorsSet(pwm, pwm);
        uint32_t tStart = millis();
        while (millis() - tStart < 400) {
            if (digitalRead(PIN_BTN_SW1) == LOW) {
                motorsBrake();
                Serial.println("[CHAR] Nguoi dung bam SW1 -> E-STOP!");
                return;
            }
            delay(10);
        }

        // 2. Giai doan do van toc on dinh (do trong 600ms tiep theo)
        long startTicksL = encodersGetLeftTicks();
        long startTicksR = encodersGetRightTicks();
        uint32_t tMeasureStartUs = micros();

        while (millis() - tStart < 1000) {
            if (digitalRead(PIN_BTN_SW1) == LOW) {
                motorsBrake();
                Serial.println("[CHAR] Nguoi dung bam SW1 -> E-STOP!");
                return;
            }
            delay(10);
        }

        uint32_t tMeasureDurationUs = micros() - tMeasureStartUs;
        long deltaTicksL = encodersGetLeftTicks() - startTicksL;
        long deltaTicksR = encodersGetRightTicks() - startTicksR;

        // Phanh ngay lap tuc sau moi nac de xe khong phong qua xa
        motorsBrake();

        float dtMeasureS = (float)tMeasureDurationUs * 1e-6f;
        measSpeedL[i] = ((float)deltaTicksL * MM_PER_TICK) / dtMeasureS;
        measSpeedR[i] = ((float)deltaTicksR * MM_PER_TICK) / dtMeasureS;

        Serial.printf("  -> PWM %3d | vL = %6.1f mm/s (+%ld ticks) | vR = %6.1f mm/s (+%ld ticks)\n",
            pwm, measSpeedL[i], deltaTicksL, measSpeedR[i], deltaTicksR);

        delay(400); // Nghi giua cac nac de nguoi dung co the dat lai xe neu can
    }

    motorsStop();

    // 3. Hoi quy tuyen tinh: PWM = ks + kv * v
    // Least squares: kv = S_vP / S_vv, ks = P_bar - kv * v_bar
    float sumP = 0, sumVL = 0, sumVR = 0;
    for (int i = 0; i < NUM_POINTS; i++) {
        sumP  += (float)TEST_PWM[i];
        sumVL += measSpeedL[i];
        sumVR += measSpeedR[i];
    }
    float avgP  = sumP / NUM_POINTS;
    float avgVL = sumVL / NUM_POINTS;
    float avgVR = sumVR / NUM_POINTS;

    float s_vvL = 0, s_vPL = 0, s_pp = 0;
    float s_vvR = 0, s_vPR = 0;
    for (int i = 0; i < NUM_POINTS; i++) {
        float devP  = (float)TEST_PWM[i] - avgP;
        float devVL = measSpeedL[i] - avgVL;
        float devVR = measSpeedR[i] - avgVR;
        s_pp  += devP * devP;
        s_vvL += devVL * devVL;
        s_vPL += devVL * devP;
        s_vvR += devVR * devVR;
        s_vPR += devVR * devP;
    }

    float kvL = (s_vvL > 1e-3f) ? (s_vPL / s_vvL) : SPEED_FF_GAIN;
    float ksL = avgP - kvL * avgVL;
    float r2L = (s_vvL > 1e-3f && s_pp > 1e-3f) ? (s_vPL * s_vPL) / (s_vvL * s_pp) : 0.0f;

    float kvR = (s_vvR > 1e-3f) ? (s_vPR / s_vvR) : SPEED_FF_GAIN;
    float ksR = avgP - kvR * avgVR;
    float r2R = (s_vvR > 1e-3f && s_pp > 1e-3f) ? (s_vPR * s_vPR) / (s_vvR * s_pp) : 0.0f;

    // Dam bao ks khong am va khong qua lon
    ksL = constrain(ksL, 5.0f, 35.0f);
    ksR = constrain(ksR, 5.0f, 35.0f);
    kvL = constrain(kvL, 0.030f, 0.200f);
    kvR = constrain(kvR, 0.030f, 0.200f);

    float maxSpeedL = (255.0f - ksL) / kvL;
    float maxSpeedR = (255.0f - ksR) / kvR;

    // Cap nhat truc tiep vao Speed Controller cho phien chay hien tai
    MotorFFParams ffL = { .ks = ksL, .kv = kvL, .ka = 0.002f, .deadzone = 0.0f };
    MotorFFParams ffR = { .ks = ksR, .kv = kvR, .ka = 0.002f, .deadzone = 0.0f };
    speedControllerSetMotorParams(ffL, ffR);

    char resBuf[32];
    snprintf(resBuf, sizeof(resBuf), "L:%.1f/%.3f R:%.1f/%.3f", ksL, kvL, ksR, kvR);
    displayShowMessage("FIT SUCCESS", resBuf, 3000);

    Serial.println("\n=======================================================");
    Serial.println("[HOI QUY DONG HOC MOTOR HOAN TAT (Least Squares Fit)]");
    Serial.printf("  BANH TRAI: ks = %5.2f PWM | kv = %.4f PWM/(mm/s) | R^2 = %.3f | V_max(255) = %.0f mm/s\n",
        ksL, kvL, r2L, maxSpeedL);
    Serial.printf("  BANH PHAI: ks = %5.2f PWM | kv = %.4f PWM/(mm/s) | R^2 = %.3f | V_max(255) = %.0f mm/s\n",
        ksR, kvR, r2R, maxSpeedR);
    Serial.println("  -> Da tu dong ap dung ks & kv vao Speed Controller cho phien chay hien tai!");
    Serial.println("=======================================================\n");
}

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
    Serial.println("  SW2 (GPIO 14): Nhan 1 lan = START RUN / STOP (Toggle)");
    Serial.println("  SW3 (GPIO 27): 1-Click = MENU CAI DAT OLED");
    Serial.println("  SERIAL (USB):  Gui 'c' = Quet dong hoc motor 5 diem & hoi quy (ks, kv)");
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
                    case MENU_ITEM_BOOST_SPD:    menuConfig.boostSpeedMmS  = constrain(menuConfig.boostSpeedMmS + 50.0f, CFG_BOOST_SPD_MIN, CFG_BOOST_SPD_MAX); break;
                    case MENU_ITEM_BASE_SPD:     menuConfig.baseSpeedMmS   = constrain(menuConfig.baseSpeedMmS + 50.0f, CFG_BASE_SPD_MIN, CFG_BASE_SPD_MAX); break;
                    case MENU_ITEM_CORNER_SPD:   menuConfig.cornerSpeedMmS = constrain(menuConfig.cornerSpeedMmS + 50.0f, CFG_CORNER_SPD_MIN, CFG_CORNER_SPD_MAX); break;
                    case MENU_ITEM_KP_GAIN:      menuConfig.kpGain         = constrain(menuConfig.kpGain + 1.0f, CFG_KP_MIN, CFG_KP_MAX); break;
                    case MENU_ITEM_KD_GAIN:      menuConfig.kdGain         = constrain(menuConfig.kdGain + 1.0f, CFG_KD_MIN, CFG_KD_MAX); break;
                    case MENU_ITEM_KC_GAIN:      menuConfig.kCurveGain     = constrain(menuConfig.kCurveGain + 0.5f, CFG_KC_MIN, CFG_KC_MAX); break;
                    case MENU_ITEM_ACTIVE_BRAKE: 
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
                    case MENU_ITEM_BOOST_SPD:    menuConfig.boostSpeedMmS  = constrain(menuConfig.boostSpeedMmS - 50.0f, CFG_BOOST_SPD_MIN, CFG_BOOST_SPD_MAX); break;
                    case MENU_ITEM_BASE_SPD:     menuConfig.baseSpeedMmS   = constrain(menuConfig.baseSpeedMmS - 50.0f, CFG_BASE_SPD_MIN, CFG_BASE_SPD_MAX); break;
                    case MENU_ITEM_CORNER_SPD:   menuConfig.cornerSpeedMmS = constrain(menuConfig.cornerSpeedMmS - 50.0f, CFG_CORNER_SPD_MIN, CFG_CORNER_SPD_MAX); break;
                    case MENU_ITEM_KP_GAIN:      menuConfig.kpGain         = constrain(menuConfig.kpGain - 1.0f, CFG_KP_MIN, CFG_KP_MAX); break;
                    case MENU_ITEM_KD_GAIN:      menuConfig.kdGain         = constrain(menuConfig.kdGain - 1.0f, CFG_KD_MIN, CFG_KD_MAX); break;
                    case MENU_ITEM_KC_GAIN:      menuConfig.kCurveGain     = constrain(menuConfig.kCurveGain - 0.5f, CFG_KC_MIN, CFG_KC_MAX); break;
                    case MENU_ITEM_ACTIVE_BRAKE: 
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
                if (menuSelectedIndex <= MENU_ITEM_ACTIVE_BRAKE) {
                    menuIsEditing = true;
                } else if (menuSelectedIndex == MENU_ITEM_SAVE_EXIT) {
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
                    motorsStop();
                    speedControllerReset();
                    displayShowMessage("LINE HOLD", "BENCH TEST MODE", 1000);
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
            brakeHoldUntilMs = now + 350; // Giữ phanh chủ động 350ms trước khi xả tự do
            motorsBrake();
            displayShowMessage("E-STOP", "EMERGENCY STOP!", 1200);
            Serial.println("[E-STOP] Emergency stop triggered by SW1!");
        }
        delay(250);
    }
    lastSw1 = sw1;

    // SW2: Single Click = RUN / STOP (Toggle)
    // Tách biệt hoàn toàn, không dùng Long Press để tránh khởi động xe ngoài ý muốn!
    if (sw2 == LOW && lastSw2 == HIGH) {
        if (appMode == MODE_RUNNING) {
            appMode = MODE_IDLE;
            speedControllerReset();
            brakeHoldUntilMs = now + 350; // Giữ phanh điện động 350ms
            motorsBrake();
            displayShowMessage("STOPPED", "CAR IN IDLE", 1000);
            Serial.printf("[STATE] Stopped. Dist L: %.1f mm | R: %.1f mm | Speed L: %.0f mm/s | R: %.0f mm/s\n",
                encodersGetLeftDistanceMm(), encodersGetRightDistanceMm(),
                encodersGetLeftSpeedMmS(), encodersGetRightSpeedMmS());
        } else if (appMode == MODE_LINE_HOLD) {
            appMode = MODE_IDLE;
            brakeHoldUntilMs = now + 350;
            motorsBrake();
            displayShowMessage("IDLE", "CAR IN IDLE", 800);
            Serial.println("[SW2] Thoat che do LINE-HOLD ve IDLE.");
        } else if (appMode == MODE_IDLE) {
            appMode = MODE_RUNNING;
            curvatureReset();
            stateMachineReset();
            encodersResetTicks();
            speedControllerReset();
            vRamp = 0.0f; // Bắt đầu đường dốc gia tốc mượt mà từ 0
            brakeHoldUntilMs = 0; // Hủy cờ phanh cũ ngay lập tức để không bị khựng
            profiledTargetSpeedMmS = activeConfig.cornerSpeedMmS;
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
        delay(200);
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
        // Kiem tra lenh Serial do ma sat tinh dong co
        if (Serial.available()) {
            char cmd = (char)Serial.read();
            if (cmd == 'c' || cmd == 'C') {
                runMotorCharacterization();
                return;
            }
        }

        if (now < brakeHoldUntilMs) {
            motorsBrake(); // Giữ phanh điện động 350ms triệt để
        } else {
            motorsStop();  // Sau 350ms xả tự do để tiết kiệm điện và chống nóng motor
        }
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
            int dir = (sensorsGetLastGoodDir() != 0) ? sensorsGetLastGoodDir() : ((runLastValidEMm >= 0.0f) ? 1 : -1);
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
        speedControllerReset();
        brakeHoldUntilMs = now + 350;
        motorsBrake();
        displayShowMessage("SAFETY STOP", "LINE LOST TIMEOUT", 2000);
        Serial.println("[SAFETY STOP] Line lost timeout exceeded! Motors shut down.");
        return;
    }

    // 4. Chọn tốc độ tuyến tính yêu cầu: Kết hợp Hysteresis đường thẳng và Phản ứng sớm de/dt
    float vTargetLinearMmS = activeConfig.baseSpeedMmS;
    if (sm.currentState == STATE_RECOVERY) {
        vTargetLinearMmS = 350.0f;
    } else if (sm.currentState == STATE_SHARP_TURN || fabsf(eMm) > 16.0f) {
        vTargetLinearMmS = activeConfig.cornerSpeedMmS;
    } else {
        // Nhận diện đường thẳng có Hysteresis:
        if (fabsf(eMm) <= 2.2f && det.lineDetected) {
            if (!inStraightTrack) {
                inStraightTrack = true;
                straightTrackStartTime = now;
            }
        } else if (fabsf(eMm) > 3.6f || !det.lineDetected) {
            inStraightTrack = false;
            straightTrackStartTime = 0;
        }

        // Tốc độ trần hành trình: Bung Boost trên đường thẳng xác nhận > 120ms
        float vCeiling = (inStraightTrack && (now - straightTrackStartTime > 120)) ?
                         activeConfig.boostSpeedMmS : activeConfig.baseSpeedMmS;

        // Giảm tốc chủ động sớm (Proactive Slowdown) theo cả sai số eMm và tốc độ đổi hướng de/dt
        // giúp xe bắt đầu hạ tốc từ 30-50ms TRƯỚC KHI góc cua làm xe vọt lố!
        vTargetLinearMmS = trackProfileComputeReactiveFallback(
            eMm,
            dErrorFiltMmS,
            det.confidence,
            vCeiling,
            activeConfig.cornerSpeedMmS,
            vMeasAvgMmS
        );
    }

    // 5. Curvature Controller: Pure Pursuit -> kappa_cmd [m^-1] -> omega [rad/s] -> v_left, v_right [mm/s]
    float kpCurv = activeConfig.kpGain;           // [m^-2] (Pure Pursuit hình học Kp: 40 - 60)
    float kdCurv = activeConfig.kdGain * 0.025f;  // [s/m^2]
    float kcCurv = activeConfig.kCurveGain;       // Thành phần phi tuyến bù góc cua
    CurvatureOutput curOut = curvatureCompute(
        eM,
        dt,
        vTargetLinearMmS,
        kpCurv,
        kdCurv,
        kcCurv,
        MAX_WHEEL_SPEED_MMS, // Trần tốc độ vật lý bánh xe (2200 mm/s), không bóp nghẹt vi sai khi chạy ở tốc độ cao!
        (sm.currentState == STATE_SHARP_TURN || fabsf(eMm) > 16.0f)
    );
    dErrorFiltMmS = curOut.dErrorMs * 1000.0f;

    // 6. Bộ giới hạn gia tốc (Slew Rate Limiter) & Trích xuất gia tốc đích thực aTargetMmS2
    // KHẮC PHỤC TRIỆT ĐỂ: Loại bỏ lỗi P-gain ẩn (ka / dt = 0.75) gây dao động bang-bang 250Hz trên đường thẳng!
    const float A_ACC = 3000.0f; // [mm/s^2] Gia tốc tiến tối đa
    const float A_DEC = 6000.0f; // [mm/s^2] Gia tốc hãm phanh tối đa (dứt khoát vào cua)
    float vReq = curOut.targetSpeedMmS;
    float step = constrain(vReq - vRamp, -A_DEC * dt, A_ACC * dt);
    vRamp += step;
    float aTargetMmS2 = step / dt;

    // Bảo toàn độ cong kappa khi điều chỉnh tốc độ qua tỷ lệ r
    float r = (vReq > 1.0f) ? (vRamp / vReq) : 1.0f;
    float vL = curOut.vLeftMmS  * r;
    float vR = curOut.vRightMmS * r;

    // 7. Speed Controller: Feedforward từng bánh + PI chung cho vận tốc tịnh tiến
    SpeedControllerOutput spdOut = speedControllerCompute(
        vRamp,
        vL,
        vR,
        aTargetMmS2,
        vMeasLMmS,
        vMeasRMmS,
        dt,
        11100.0f, // [GIẢ ĐỊNH] Vbat thực tế ~ 11.1V
        11100.0f, // Vbat danh định 11.1V
        (sm.currentState != STATE_RUN) // Freeze integral trong cua gắt / recovery
    );

    // 8. Xuất tín hiệu động cơ qua DRV8871
    motorsSet(spdOut.pwmL, spdOut.pwmR);

    // 9. Giám sát hiệu năng và Telemetry 250 Hz
    static uint32_t lastLoopUs = 0;
    static uint32_t maxLoopUs = 0;
    static uint32_t lastLoopStatMs = 0;
    uint32_t loopNowUs = micros();
    if (lastLoopUs > 0) {
        uint32_t loopDelta = loopNowUs - lastLoopUs;
        if (loopDelta > maxLoopUs) maxLoopUs = loopDelta;
    }
    lastLoopUs = loopNowUs;

    // Tính gia tốc thực tế đo được từ encoder để kiểm chứng độ hãm thực của động cơ trên mặt sàn
    static float lastVMeasAvg = 0.0f;
    static float aMeasFiltMmS2 = 0.0f;
    float aMeasRaw = (vMeasAvgMmS - lastVMeasAvg) / dt;
    lastVMeasAvg = vMeasAvgMmS;
    aMeasFiltMmS2 += 0.20f * (aMeasRaw - aMeasFiltMmS2);

    if (now - lastLoopStatMs >= 1000) {
        lastLoopStatMs = now;
        if (Serial.availableForWrite() >= 96) {
            Serial.printf("[PERF] LoopMax:%u us | e:%+.1f mm | kap:%+.2f | vRamp:%.0f (L:%.0f R:%.0f) | aTgt:%+.0f aAct:%+.0f | pwmL:%+d pwmR:%+d\n",
                maxLoopUs, eMm, curOut.kappaCmd, vRamp, vMeasLMmS, vMeasRMmS, aTargetMmS2, aMeasFiltMmS2, spdOut.pwmL, spdOut.pwmR);
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
            vRamp
        );
    }
}