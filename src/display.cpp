#include "display.h"
#include "config.h"
#include <Wire.h>
#include <U8g2lib.h>

// ----------------------------------------------------
// U8g2 Display Constructor for 1.3" I2C OLED (128x64)
// ----------------------------------------------------
#if OLED_DRIVER_SH1106
static U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ PIN_I2C_SCL, /* data=*/ PIN_I2C_SDA);
#else
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ PIN_I2C_SCL, /* data=*/ PIN_I2C_SDA);
#endif

struct DisplaySnapshotData {
    DisplayMode mode;
    float error;
    bool lineDetected;
    int sensorNorm[8];
    float leftSpeedMmS;
    float rightSpeedMmS;
    float leftDistMm;
    float rightDistMm;
    int leftPwm;
    int rightPwm;
    float rate;
    bool isCalibrated;
    float baseSpeed;
    uint32_t calibElapsedMs;
    uint32_t calibTotalMs;
    char msgTitle[20];
    char msgText[28];
    uint32_t msgUntilMs;
    MenuDisplayData menu;
};

static DisplaySnapshotData currentSnapshot = {
    DISP_MODE_IDLE, 0.0f, false, {0}, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0.0f, false, 140.0f, 0, 0, "", "", 0,
    {0, false, DEFAULT_BOOST_SPEED, DEFAULT_BASE_SPEED, DEFAULT_CORNER_SPEED, KP_GAIN, KD_GAIN, K_CURVE_GAIN, true}
};

static portMUX_TYPE dispMux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t oledTaskHandle = NULL;

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
    float baseSpeed
) {
    portENTER_CRITICAL(&dispMux);
    currentSnapshot.mode          = mode;
    currentSnapshot.error         = error;
    currentSnapshot.lineDetected  = lineDetected;
    if (sensorNorm != nullptr) {
        for (int i = 0; i < 8; i++) {
            currentSnapshot.sensorNorm[i] = sensorNorm[i];
        }
    }
    currentSnapshot.leftSpeedMmS  = leftSpeedMmS;
    currentSnapshot.rightSpeedMmS = rightSpeedMmS;
    currentSnapshot.leftDistMm    = leftDistMm;
    currentSnapshot.rightDistMm   = rightDistMm;
    currentSnapshot.leftPwm       = leftPwm;
    currentSnapshot.rightPwm      = rightPwm;
    currentSnapshot.rate          = rate;
    currentSnapshot.isCalibrated  = isCalibrated;
    currentSnapshot.baseSpeed     = baseSpeed;
    portEXIT_CRITICAL(&dispMux);
}

void displayUpdateMenu(const MenuDisplayData &menu) {
    portENTER_CRITICAL(&dispMux);
    currentSnapshot.mode       = DISP_MODE_MENU;
    currentSnapshot.msgUntilMs = 0;
    currentSnapshot.msgTitle[0] = '\0';
    currentSnapshot.msgText[0]  = '\0';
    currentSnapshot.menu       = menu;
    portEXIT_CRITICAL(&dispMux);
}

void displaySetCalibProgress(uint32_t elapsedMs, uint32_t totalMs) {
    portENTER_CRITICAL(&dispMux);
    currentSnapshot.mode           = DISP_MODE_CALIBRATING;
    currentSnapshot.calibElapsedMs = elapsedMs;
    currentSnapshot.calibTotalMs   = (totalMs > 0) ? totalMs : 10000;
    portEXIT_CRITICAL(&dispMux);
}

void displayShowMessage(const char *title, const char *msg, uint32_t durationMs) {
    portENTER_CRITICAL(&dispMux);
    strncpy(currentSnapshot.msgTitle, title ? title : "", sizeof(currentSnapshot.msgTitle) - 1);
    currentSnapshot.msgTitle[sizeof(currentSnapshot.msgTitle) - 1] = '\0';
    strncpy(currentSnapshot.msgText, msg ? msg : "", sizeof(currentSnapshot.msgText) - 1);
    currentSnapshot.msgText[sizeof(currentSnapshot.msgText) - 1] = '\0';
    currentSnapshot.msgUntilMs = millis() + durationMs;
    portEXIT_CRITICAL(&dispMux);
}

void displayClearMessage() {
    portENTER_CRITICAL(&dispMux);
    currentSnapshot.msgUntilMs = 0;
    currentSnapshot.msgTitle[0] = '\0';
    currentSnapshot.msgText[0] = '\0';
    portEXIT_CRITICAL(&dispMux);
}

// ----------------------------------------------------
// UI Render Screens (128x64 Monochrome)
// ----------------------------------------------------

static void drawSplash() {
    u8g2.clearBuffer();
    u8g2.drawFrame(0, 0, 128, 64);
    u8g2.drawFrame(2, 2, 124, 60);

    u8g2.setFont(u8g2_font_7x14_tf);
    u8g2.drawStr(10, 20, "FAST LINE CAR");

    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(12, 36, "ESP32 | 250Hz PD");
    u8g2.drawStr(12, 50, "1.3\" SH1106 OLED");

    u8g2.sendBuffer();
}

static void drawIdle(const DisplaySnapshotData &data) {
    // Top Bar (Inverted header)
    u8g2.drawBox(0, 0, 128, 11);
    u8g2.setDrawColor(0);
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(2, 9, "IDLE | SW2:RUN SW3:MENU");
    u8g2.setDrawColor(1);

    // 8-Sensor Bar Graph
    const int startX = 13;
    const int startY = 14;
    for (int i = 0; i < 8; i++) {
        int x = startX + i * 13;
        if (data.sensorNorm[i] > 180) {
            u8g2.drawBox(x, startY, 11, 10);
        } else {
            u8g2.drawFrame(x, startY, 11, 10);
        }
    }

    // Position pointer triangle below sensor boxes
    if (data.lineDetected) {
        int needleX = 64 + (int)roundf(data.error * 1.15f);
        needleX = constrain(needleX, 10, 117);
        u8g2.drawTriangle(needleX, 26, needleX - 3, 30, needleX + 3, 30);
    }

    // Telemetry text
    u8g2.setFont(u8g2_font_6x10_tf);
    char buf[32];

    snprintf(buf, sizeof(buf), "e:%+4.1fmm  [%s]", data.error, data.isCalibrated ? "CAL:OK" : "NO-CAL");
    u8g2.drawStr(2, 42, buf);

    snprintf(buf, sizeof(buf), "L:%4.0f mm  R:%4.0f mm", data.leftDistMm, data.rightDistMm);
    u8g2.drawStr(2, 52, buf);

    snprintf(buf, sizeof(buf), "Base:%.0f [SW2-GIU:HOLD]", data.baseSpeed > 0.0f ? data.baseSpeed : DEFAULT_BASE_SPEED);
    u8g2.drawStr(2, 62, buf);
}

static void drawRunning(const DisplaySnapshotData &data) {
    // Top Bar (Inverted header)
    u8g2.drawBox(0, 0, 128, 11);
    u8g2.setDrawColor(0);
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(2, 9, "RUNNING | 250Hz PD");
    u8g2.setDrawColor(1);

    // Deviation gauge bar
    u8g2.drawFrame(14, 14, 100, 8);
    u8g2.drawLine(64, 13, 64, 22); // Center target line

    if (data.lineDetected) {
        int markX = 64 + (int)roundf(data.error * 1.15f);
        markX = constrain(markX, 16, 110);
        u8g2.drawBox(markX - 2, 15, 5, 6);
    } else {
        u8g2.setFont(u8g2_font_5x7_tf);
        u8g2.drawStr(42, 21, "! LOST !");
    }

    // Telemetry metrics
    u8g2.setFont(u8g2_font_6x10_tf);
    char buf[32];

    snprintf(buf, sizeof(buf), "e:%+4.1fmm  %s", data.error, data.lineDetected ? "TRACK" : "SEARCH");
    u8g2.drawStr(2, 35, buf);

    snprintf(buf, sizeof(buf), "PWM: L:%+4d R:%+4d", data.leftPwm, data.rightPwm);
    u8g2.drawStr(2, 48, buf);

    snprintf(buf, sizeof(buf), "Spd: %-4.0f Dist:%-5.0f", (data.leftSpeedMmS + data.rightSpeedMmS) * 0.5f, (data.leftDistMm + data.rightDistMm) * 0.5f);
    u8g2.drawStr(2, 61, buf);
}

static void drawHold(const DisplaySnapshotData &data) {
    // Top Bar (Inverted header)
    u8g2.drawBox(0, 0, 128, 11);
    u8g2.setDrawColor(0);
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(2, 9, "LINE-HOLD BENCH TEST");
    u8g2.setDrawColor(1);

    // Deviation gauge bar
    u8g2.drawFrame(14, 14, 100, 8);
    u8g2.drawLine(64, 13, 64, 22);

    if (data.lineDetected) {
        int markX = 64 + (int)roundf(data.error * 1.15f);
        markX = constrain(markX, 16, 110);
        u8g2.drawBox(markX - 2, 15, 5, 6);
    }

    u8g2.setFont(u8g2_font_6x10_tf);
    char buf[32];

    snprintf(buf, sizeof(buf), "e:%+4.1fmm  Rt:%+4.0f", data.error, data.rate);
    u8g2.drawStr(2, 35, buf);

    snprintf(buf, sizeof(buf), "PWM: L:%+4d  R:%+4d", data.leftPwm, data.rightPwm);
    u8g2.drawStr(2, 48, buf);

    const char *statusStr = "TRACKING";
    if (!data.lineDetected) {
        statusStr = "LINE LOST!";
    } else if (fabsf(data.error) < 1.0f) {
        statusStr = "CENTERED";
    }
    snprintf(buf, sizeof(buf), "State: %s", statusStr);
    u8g2.drawStr(2, 61, buf);
}

static void drawCalib(const DisplaySnapshotData &data) {
    u8g2.drawBox(0, 0, 128, 11);
    u8g2.setDrawColor(0);
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(2, 9, "CALIBRATING (10s)");
    u8g2.setDrawColor(1);

    u8g2.drawFrame(10, 16, 108, 12);
    int pct = 0;
    if (data.calibTotalMs > 0) {
        pct = (int)((data.calibElapsedMs * 100UL) / data.calibTotalMs);
    }
    pct = constrain(pct, 0, 100);
    u8g2.drawBox(12, 18, (pct * 104) / 100, 8);

    u8g2.setFont(u8g2_font_6x10_tf);
    char buf[32];
    snprintf(buf, sizeof(buf), "Progress: %d%% (%u s)", pct, (data.calibElapsedMs / 1000));
    u8g2.drawStr(12, 40, buf);

    u8g2.drawStr(12, 51, "Truot xe qua lai");
    u8g2.drawStr(12, 61, "vach de quet 8 mat");
}

static void drawPopup(const char *title, const char *msg) {
    u8g2.drawFrame(6, 10, 116, 46);
    u8g2.drawBox(8, 12, 112, 42);
    u8g2.setDrawColor(0);

    u8g2.setFont(u8g2_font_7x14_tf);
    u8g2.drawStr(14, 28, title);

    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(14, 46, msg);

    u8g2.setDrawColor(1);
}

static void drawMenu(const MenuDisplayData &menu) {
    const int VISIBLE_ROWS = 4;

    u8g2.drawBox(0, 0, 128, 11);
    u8g2.setDrawColor(0);
    u8g2.setFont(u8g2_font_6x10_tf);
    if (menu.isEditing) {
        u8g2.drawStr(2, 9, "[EDIT] 1:+ 2:- 3:OK");
    } else {
        u8g2.drawStr(2, 9, "MENU | 1:^ 2:v 3:SEL");
    }
    u8g2.setDrawColor(1);

    int topIndex = 0;
    if (menu.selectedIndex >= 3) {
        topIndex = menu.selectedIndex - 2;
    }
    if (topIndex > TOTAL_MENU_ITEMS - VISIBLE_ROWS) {
        topIndex = TOTAL_MENU_ITEMS - VISIBLE_ROWS;
    }
    if (topIndex < 0) {
        topIndex = 0;
    }

    for (int row = 0; row < VISIBLE_ROWS; row++) {
        int itemIdx = topIndex + row;
        if (itemIdx >= TOTAL_MENU_ITEMS) break;

        int y = 23 + row * 13;

        char nameBuf[22];
        char valBuf[16];
        nameBuf[0] = '\0';
        valBuf[0] = '\0';

        switch (itemIdx) {
            case MENU_ITEM_BOOST_SPD:
                snprintf(nameBuf, sizeof(nameBuf), "Bst  Spd");
                snprintf(valBuf, sizeof(valBuf), "%4.0f", menu.boostSpeed);
                break;
            case MENU_ITEM_BASE_SPD:
                snprintf(nameBuf, sizeof(nameBuf), "Base Spd");
                snprintf(valBuf, sizeof(valBuf), "%4.0f", menu.baseSpeed);
                break;
            case MENU_ITEM_CORNER_SPD:
                snprintf(nameBuf, sizeof(nameBuf), "Cnr  Spd");
                snprintf(valBuf, sizeof(valBuf), "%4.0f", menu.cornerSpeed);
                break;
            case MENU_ITEM_KP_GAIN:
                snprintf(nameBuf, sizeof(nameBuf), "Kp Gain");
                snprintf(valBuf, sizeof(valBuf), "%4.1f", menu.kpGain);
                break;
            case MENU_ITEM_KD_GAIN:
                snprintf(nameBuf, sizeof(nameBuf), "Kd Gain");
                snprintf(valBuf, sizeof(valBuf), "%4.1f", menu.kdGain);
                break;
            case MENU_ITEM_KC_GAIN:
                snprintf(nameBuf, sizeof(nameBuf), "K-Curve");
                snprintf(valBuf, sizeof(valBuf), "%4.1f", menu.kCurveGain);
                break;
            case MENU_ITEM_ACTIVE_BRAKE:
                snprintf(nameBuf, sizeof(nameBuf), "ABS Brk");
                snprintf(valBuf, sizeof(valBuf), "%s", menu.activeBrakeEn ? "ON" : "OFF");
                break;
            case MENU_ITEM_SAVE_EXIT:
                snprintf(nameBuf, sizeof(nameBuf), "[SAVE & EXIT]");
                break;
            case MENU_ITEM_CANCEL:
                snprintf(nameBuf, sizeof(nameBuf), "[CANCEL]");
                break;
            case MENU_ITEM_RESTORE_DEF:
                snprintf(nameBuf, sizeof(nameBuf), "[RESTORE DEF]");
                break;
            case MENU_ITEM_TEST_LINE_HOLD:
                snprintf(nameBuf, sizeof(nameBuf), "[TEST LINE-HOLD]");
                break;
        }

        bool isSelected = (itemIdx == menu.selectedIndex);
        if (isSelected) {
            if (menu.isEditing) {
                u8g2.drawFrame(0, y - 10, 123, 12);
                u8g2.drawStr(2, y, "*");
                u8g2.drawStr(10, y, nameBuf);
                if (valBuf[0] != '\0') {
                    char editVal[20];
                    snprintf(editVal, sizeof(editVal), "<%s>", valBuf);
                    u8g2.drawStr(74, y, editVal);
                }
            } else {
                u8g2.drawBox(0, y - 10, 123, 12);
                u8g2.setDrawColor(0);
                u8g2.drawStr(2, y, ">");
                u8g2.drawStr(10, y, nameBuf);
                if (valBuf[0] != '\0') {
                    u8g2.drawStr(80, y, valBuf);
                }
                u8g2.setDrawColor(1);
            }
        } else {
            u8g2.drawStr(10, y, nameBuf);
            if (valBuf[0] != '\0') {
                u8g2.drawStr(80, y, valBuf);
            }
        }
    }

    u8g2.drawVLine(126, 12, 51);
    int barHeight = 20;
    int barTop = 13 + (topIndex * (50 - barHeight)) / (TOTAL_MENU_ITEMS - VISIBLE_ROWS);
    u8g2.drawBox(125, barTop, 3, barHeight);
}

// ----------------------------------------------------
// FreeRTOS Core 0 Background Task
// ----------------------------------------------------
static void oledCore0Task(void *pvParameters) {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
    u8g2.begin();
    u8g2.setBusClock(400000);

    drawSplash();
    vTaskDelay(pdMS_TO_TICKS(1200));

    while (true) {
        DisplaySnapshotData snap;
        portENTER_CRITICAL(&dispMux);
        snap = currentSnapshot;
        portEXIT_CRITICAL(&dispMux);

        u8g2.clearBuffer();

        uint32_t now = millis();
        if (snap.mode == DISP_MODE_MENU) {
            drawMenu(snap.menu);
        } else if (now < snap.msgUntilMs) {
            drawPopup(snap.msgTitle, snap.msgText);
        } else {
            switch (snap.mode) {
                case DISP_MODE_IDLE:
                    drawIdle(snap);
                    break;
                case DISP_MODE_RUNNING:
                    drawRunning(snap);
                    break;
                case DISP_MODE_LINE_HOLD:
                    drawHold(snap);
                    break;
                case DISP_MODE_CALIBRATING:
                    drawCalib(snap);
                    break;
                case DISP_MODE_MENU:
                    drawMenu(snap.menu);
                    break;
            }
        }

        u8g2.sendBuffer();
        vTaskDelay(pdMS_TO_TICKS(1000 / OLED_REFRESH_HZ));
    }
}

void displayInit() {
    xTaskCreatePinnedToCore(
        oledCore0Task,
        "OledCore0Task",
        8192,
        NULL,
        1,
        &oledTaskHandle,
        0
    );
}
