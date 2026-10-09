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
    float baseSpeedMmS;
    uint32_t calibElapsedMs;
    uint32_t calibTotalMs;
    char msgTitle[20];
    char msgText[28];
    uint32_t msgUntilMs;
    MenuDisplayData menu;
};

static DisplaySnapshotData currentSnapshot = {
    DISP_MODE_IDLE, 0.0f, false, {0}, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0.0f, false, 1250.0f, 0, 0, "", "", 0,
    {0, false, 1650.0f, 1250.0f, 850.0f, 24.0f, 38.0f, 7.5f, true}
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
    float baseSpeedMmS
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
    currentSnapshot.baseSpeedMmS  = baseSpeedMmS;
    portEXIT_CRITICAL(&dispMux);
}

void displayUpdateMenu(const MenuDisplayData &menu) {
    portENTER_CRITICAL(&dispMux);
    currentSnapshot.mode       = DISP_MODE_MENU;
    currentSnapshot.msgUntilMs = 0; // Xóa ngay lập tức bất kỳ popup nào đang chạy
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
    u8g2.drawStr(12, 36, "ESP32 | 250Hz Loop");
    u8g2.drawStr(12, 50, "1.3\" SH1106 OLED");

    u8g2.sendBuffer();
}

static void drawIdle(const DisplaySnapshotData &data) {
    // Top Bar (Inverted header)
    u8g2.drawBox(0, 0, 128, 11);
    u8g2.setDrawColor(0);
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(2, 9, "IDLE | SW2:RUN SW1:CAL");
    u8g2.setDrawColor(1);

    // 8-Sensor Bar Graph
    // 8 boxes: width 11px, gap 2px. Total width = 8*11 + 7*2 = 102px
    const int startX = 13;
    const int startY = 14;
    for (int i = 0; i < 8; i++) {
        int x = startX + i * 13;
        if (data.sensorNorm[i] > 250) {
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

    snprintf(buf, sizeof(buf), "Base:%.0f [SW3:MENU]", data.baseSpeedMmS > 0.0f ? data.baseSpeedMmS : TARGET_BASE_SPEED_MMS);
    u8g2.drawStr(2, 62, buf);
}

static void drawRunning(const DisplaySnapshotData &data) {
    // Top Bar (Inverted header)
    u8g2.drawBox(0, 0, 128, 11);
    u8g2.setDrawColor(0);
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(2, 9, "RUNNING | 250Hz CASCADE");
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

    snprintf(buf, sizeof(buf), "Spd: L:%-4.0f R:%-4.0f", data.leftSpeedMmS, data.rightSpeedMmS);
    u8g2.drawStr(2, 48, buf);

    snprintf(buf, sizeof(buf), "Dist: %-5.0f mm", (data.leftDistMm + data.rightDistMm) * 0.5f);
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
    u8g2.drawLine(64, 13, 64, 22); // Center target line

    if (data.lineDetected) {
        int markX = 64 + (int)roundf(data.error * 1.15f);
        markX = constrain(markX, 16, 110);
        u8g2.drawBox(markX - 2, 15, 5, 6);
    }

    // Control parameters
    u8g2.setFont(u8g2_font_6x10_tf);
    char buf[32];

    snprintf(buf, sizeof(buf), "e:%+4.1fmm  Rt:%+4.0f", data.error, data.rate);
    u8g2.drawStr(2, 35, buf);

    snprintf(buf, sizeof(buf), "PWM: L:%+4d  R:%+4d", data.leftPwm, data.rightPwm);
    u8g2.drawStr(2, 48, buf);

    const char *statusStr = "ZONE 2 (DAMP)";
    if (!data.lineDetected) {
        statusStr = "LINE LOST!";
    } else if (fabsf(data.error) < 1.0f) {
        statusStr = "ZONE 3 (CENTERED)";
    } else if (fabsf(data.error) >= 6.0f) {
        statusStr = "ZONE 1 (MAX P)";
    }
    snprintf(buf, sizeof(buf), "State: %s", statusStr);
    u8g2.drawStr(2, 61, buf);
}

static void drawCalib(const DisplaySnapshotData &data) {
    // Top Bar (Inverted header)
    u8g2.drawBox(0, 0, 128, 11);
    u8g2.setDrawColor(0);
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(2, 9, "CALIBRATING (10s)");
    u8g2.setDrawColor(1);

    // Progress Bar
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

    // Inverted Top Header Bar
    u8g2.drawBox(0, 0, 128, 11);
    u8g2.setDrawColor(0);
    u8g2.setFont(u8g2_font_6x10_tf);
    if (menu.isEditing) {
        u8g2.drawStr(2, 9, "[EDIT] 1:+ 2:- 3:OK");
    } else {
        u8g2.drawStr(2, 9, "MENU | 1:^ 2:v 3:SEL");
    }
    u8g2.setDrawColor(1);

    // Scroll window calculation
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

    // Render 4 visible rows
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
                snprintf(valBuf, sizeof(valBuf), "%4.0f", menu.boostSpeedMmS);
                break;
            case MENU_ITEM_BASE_SPD:
                snprintf(nameBuf, sizeof(nameBuf), "Base Spd");
                snprintf(valBuf, sizeof(valBuf), "%4.0f", menu.baseSpeedMmS);
                break;
            case MENU_ITEM_CORNER_SPD:
                snprintf(nameBuf, sizeof(nameBuf), "Cnr  Spd");
                snprintf(valBuf, sizeof(valBuf), "%4.0f", menu.cornerSpeedMmS);
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
                // Editing mode: hollow frame with edit arrows
                u8g2.drawFrame(0, y - 10, 123, 12);
                u8g2.drawStr(2, y, "*");
                u8g2.drawStr(10, y, nameBuf);
                if (valBuf[0] != '\0') {
                    char editVal[20];
                    snprintf(editVal, sizeof(editVal), "<%s>", valBuf);
                    u8g2.drawStr(74, y, editVal);
                }
            } else {
                // Navigation mode: solid inverted bar
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
            // Normal unselected row
            u8g2.drawStr(10, y, nameBuf);
            if (valBuf[0] != '\0') {
                u8g2.drawStr(80, y, valBuf);
            }
        }
    }

    // Scrollbar on right edge (x=126, y=12..63)
    u8g2.drawVLine(126, 12, 51);
    int barHeight = 20;
    int barTop = 13 + (topIndex * (50 - barHeight)) / (TOTAL_MENU_ITEMS - VISIBLE_ROWS);
    u8g2.drawBox(125, barTop, 3, barHeight);
}

// ----------------------------------------------------
// FreeRTOS Core 0 Background Task
// ----------------------------------------------------
static void oledCore0Task(void *pvParameters) {
    // Initialize I2C hardware bus on GPIO 21 (SDA) and GPIO 22 (SCL) at 400 kHz
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
    u8g2.begin();
    u8g2.setBusClock(400000);

    // Boot Splash Screen
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
            // Chế độ MENU có độ ưu tiên cao nhất, tuyệt đối KHÔNG bị popup che khuất!
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

        // 10 Hz refresh rate (100 ms period)
        vTaskDelay(pdMS_TO_TICKS(1000 / OLED_REFRESH_HZ));
    }
}

void displayInit() {
    xTaskCreatePinnedToCore(
        oledCore0Task,      // Task function
        "OledCore0Task",    // Name of task
        8192,               // Stack size in bytes (tăng lên 8192 chống tràn stack do snprintf float & U8g2)
        NULL,               // Parameters
        1,                  // Low priority (1)
        &oledTaskHandle,    // Task handle
        0                   // Core 0 (Protects Core 1 250Hz real-time loop!)
    );
}
