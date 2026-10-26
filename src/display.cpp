#include "display.h"
#include "config.h"
#include "params.h"
#include <Wire.h>
#include <U8g2lib.h>
#include <Preferences.h>

#if OLED_DRIVER_SH1106
static U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ PIN_I2C_SCL, /* data=*/ PIN_I2C_SDA);
#else
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ PIN_I2C_SCL, /* data=*/ PIN_I2C_SDA);
#endif

// ------------------------------------------------------------
// Bảng định nghĩa 10 tham số cốt lõi trên Menu OLED:
// Tên, Mặc định, Min, Max, Bước chỉnh, Số chữ số thập phân (253 = Hiển thị % hoặc OFF)
// Đã sửa Min để phản ánh chính xác dải điều khiển PWM phần cứng:
// - Bst: min 80 (x16 = 1280 PWM)
// - Base: min 70 (x16 = 1120 PWM)
// - Cnr: min 85 (x16 = 1360 PWM > sàn minTorqueFloor 1350)
// ------------------------------------------------------------
#define PCT_DEC 253
#define ONOFF_DEC 254
#define PREFS_SCHEMA_VER 6

struct ParamDef {
  const char *name;
  float def, mn, mx, step;
  uint8_t dec;
};

static const ParamDef PDEF[P_COUNT] = {
  {"Bst Spd",  240.0f,   80.0f, 255.0f,   5.0f, 0},       // P_BOOST   : Tốc độ tối đa SW3
  {"Base Spd", 145.0f,   70.0f, 255.0f,   5.0f, 0},       // P_BASE    : Tốc độ chuẩn SW2
  {"Cnr Spd",   95.0f,   85.0f, 220.0f,   5.0f, 0},       // P_CORNER  : Tốc độ ôm cua
  {"Kp Gain",   15.0f,    1.0f,  50.0f,   0.5f, 1},       // P_KP      : Hệ số lái P
  {"Kd Gain",   60.0f,    0.0f, 150.0f,   2.0f, 0},       // P_KD      : Hệ số dập dao động D
  {"K-Curve",    6.5f,    0.0f,  30.0f,   0.5f, 1},       // P_KCURVE  : Tăng lực cua gắt & xoay 90°
  {"Accel",     30.0f,    5.0f, 100.0f,   5.0f, 0},       // P_ACCEL   : Gia tốc đường thẳng mỗi 4ms
  {"Brk Pwr",   80.0f,    0.0f, 100.0f,   5.0f, PCT_DEC}, // P_BRK_PWR : Cường độ phanh (0%=OFF, 80%=Chuẩn)
  {"Brk Pos", 2600.0f, 1500.0f, 6000.0f, 100.0f, 0},      // P_BRK_POS : Ngưỡng lệch vạch kích hoạt phanh
  {"W-Loop",     1.0f,    0.0f,   1.0f,   1.0f, ONOFF_DEC} // P_WLOOP  : Vòng lặp kín vận tốc bánh xe
};

static const int N_ACTIONS  = 4;
static const int N_ITEMS    = P_COUNT + N_ACTIONS;
static const int IDX_SAVE    = P_COUNT;
static const int IDX_CANCEL  = P_COUNT + 1;
static const int IDX_RESTORE = P_COUNT + 2;
static const int IDX_TEST    = P_COUNT + 3;
static const char *ACTION_NAME[N_ACTIONS] = {
  "[SAVE & EXIT]", "[CANCEL]", "[RESTORE DEF]", "[TEST HOLD]"
};

static float mCfg[P_COUNT];   // Bản đang chỉnh trong Menu
static bool s_inMenu = false;
static int  s_menuIndex = 0;
static bool s_menuEdit = false;
static bool s_testHold = false;

static void setDefaults() {
  for (int i = 0; i < P_COUNT; i++) {
    mCfg[i] = PDEF[i].def;
  }
}

static void syncToParams() {
  for (int i = 0; i < P_COUNT; i++) {
    gMenuParams[i] = constrain(mCfg[i], PDEF[i].mn, PDEF[i].mx);
  }
  paramsSyncFromMenu();
}

static void stepParam(int idx, int dir) {
  const ParamDef &d = PDEF[idx];
  float v = mCfg[idx] + dir * d.step;
  v = constrain(v, d.mn, d.mx);
  v = roundf(v / d.step) * d.step;
  mCfg[idx] = constrain(v, d.mn, d.mx);
}

static void formatValue(int idx, char *out, size_t n) {
  const ParamDef &d = PDEF[idx];
  if (d.dec == PCT_DEC) {
    if (mCfg[idx] < 5.0f) {
      snprintf(out, n, "OFF");
    } else {
      snprintf(out, n, "%.0f%%", mCfg[idx]);
    }
  } else if (d.dec == ONOFF_DEC) {
    snprintf(out, n, (mCfg[idx] >= 0.5f) ? "ON" : "OFF");
  } else {
    snprintf(out, n, "%.*f", (int)d.dec, mCfg[idx]);
  }
}

static void savePrefs() {
  Preferences p;
  if (!p.begin("menu2", false)) return;
  char key[8];
  for (int i = 0; i < P_COUNT; i++) {
    snprintf(key, sizeof(key), "v%d", i);
    p.putFloat(key, mCfg[i]);
  }
  p.putUChar("ver", PREFS_SCHEMA_VER);
  p.end();
  syncToParams();
}

static void loadPrefs() {
  setDefaults();
  Preferences p;
  bool needSave = false;
  if (p.begin("menu2", true)) {
    uint8_t ver = p.getUChar("ver", 0);
    char key[8];
    if (ver == PREFS_SCHEMA_VER) {
      for (int i = 0; i < P_COUNT; i++) {
        snprintf(key, sizeof(key), "v%d", i);
        float v = p.getFloat(key, PDEF[i].def);
        mCfg[i] = constrain(v, PDEF[i].mn, PDEF[i].mx);
      }
    } else if (ver == 5) {
      // Migration 5 -> 6: giữ nguyên 9 thông số đã tuning, thêm W-Loop mặc định ON
      for (int i = 0; i < P_WLOOP; i++) {
        snprintf(key, sizeof(key), "v%d", i);
        float v = p.getFloat(key, PDEF[i].def);
        mCfg[i] = constrain(v, PDEF[i].mn, PDEF[i].mx);
      }
      mCfg[P_WLOOP] = PDEF[P_WLOOP].def;
      needSave = true;
    }
    p.end();
  } else {
    needSave = true;
  }
  if (needSave) {
    savePrefs();
  } else {
    syncToParams();
  }
}

void displayInit() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  u8g2.begin();
  u8g2.setBusClock(400000);
  u8g2.setFont(u8g2_font_ncenB08_tr);
  loadPrefs();
  s_inMenu = false;
}

void displayMenuLoop() {
  const uint32_t REPEAT_DELAY_MS = 400;
  const uint32_t HOLD_FAST_MS    = 1500;

  s_inMenu = true;
  s_menuEdit = false;
  bool last1 = (digitalRead(PIN_BTN_SW1) == LOW);
  bool last2 = (digitalRead(PIN_BTN_SW2) == LOW);
  bool last3 = (digitalRead(PIN_BTN_SW3) == LOW);
  uint32_t press1 = 0, press2 = 0, rep1 = 0, rep2 = 0;
  bool needRedraw = true;

  while (s_inMenu) {
    uint32_t now = millis();
    bool sw1 = (digitalRead(PIN_BTN_SW1) == LOW);
    bool sw2 = (digitalRead(PIN_BTN_SW2) == LOW);
    bool sw3 = (digitalRead(PIN_BTN_SW3) == LOW);

    bool first1 = sw1 && !last1;
    bool first2 = sw2 && !last2;
    bool fire1 = false, fire2 = false;
    int mult1 = 1, mult2 = 1;

    if (sw1) {
      if (first1) { fire1 = true; press1 = now; rep1 = now + REPEAT_DELAY_MS; }
      else if ((int32_t)(now - rep1) >= 0) { fire1 = true; rep1 = now + (s_menuEdit ? 70 : 150); }
      if (now - press1 > HOLD_FAST_MS) mult1 = 5;
    }
    if (sw2) {
      if (first2) { fire2 = true; press2 = now; rep2 = now + REPEAT_DELAY_MS; }
      else if ((int32_t)(now - rep2) >= 0) { fire2 = true; rep2 = now + (s_menuEdit ? 70 : 150); }
      if (now - press2 > HOLD_FAST_MS) mult2 = 5;
    }

    if (fire1) {
      needRedraw = true;
      if (!s_menuEdit) {
        s_menuIndex = (s_menuIndex + N_ITEMS - 1) % N_ITEMS;
      } else if (s_menuIndex < P_COUNT) {
        stepParam(s_menuIndex, +mult1);
      }
    }
    if (fire2) {
      needRedraw = true;
      if (!s_menuEdit) {
        s_menuIndex = (s_menuIndex + 1) % N_ITEMS;
      } else if (s_menuIndex < P_COUNT) {
        stepParam(s_menuIndex, -mult2);
      }
    }

    if (sw3 && !last3) {
      needRedraw = true;
      if (s_menuEdit) {
        s_menuEdit = false;
      } else if (s_menuIndex < P_COUNT) {
        s_menuEdit = true;
      } else if (s_menuIndex == IDX_SAVE) {
        savePrefs();
        s_inMenu = false;
      } else if (s_menuIndex == IDX_CANCEL) {
        loadPrefs();
        s_inMenu = false;
      } else if (s_menuIndex == IDX_RESTORE) {
        setDefaults();
        savePrefs();
      } else if (s_menuIndex == IDX_TEST) {
        syncToParams();
        s_testHold = true;
        s_inMenu = false;
      }
    }
    last1 = sw1; last2 = sw2; last3 = sw3;

    if (needRedraw) {
      needRedraw = false;
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.drawStr(0, 10, s_menuEdit ? "[EDIT] 1:+ 2:- 3:OK" : "MENU | 1:^ 2:v 3:SEL");

      int top = constrain(s_menuIndex - 2, 0, N_ITEMS - 4);
      char buf[28];
      char val[16];
      for (int r = 0; r < 4; r++) {
        int idx = top + r;
        if (idx >= N_ITEMS) break;
        bool sel = (idx == s_menuIndex);
        int y = 22 + r * 12;

        if (idx < P_COUNT) {
          snprintf(buf, sizeof(buf), "%c %s", sel ? '>' : ' ', PDEF[idx].name);
          u8g2.drawStr(0, y, buf);

          formatValue(idx, val, sizeof(val));
          if (s_menuEdit && sel) {
            char t[20];
            snprintf(t, sizeof(t), "<%s>", val);
            u8g2.drawStr(78, y, t);
          } else {
            u8g2.drawStr(78, y, val);
          }
        } else {
          snprintf(buf, sizeof(buf), "%c %s", sel ? '>' : ' ', ACTION_NAME[idx - P_COUNT]);
          u8g2.drawStr(0, y, buf);
        }
      }
      u8g2.sendBuffer();
    }
    delay(25);
  }

  uint32_t releaseWait = millis();
  while ((digitalRead(PIN_BTN_SW1) == LOW ||
          digitalRead(PIN_BTN_SW2) == LOW ||
          digitalRead(PIN_BTN_SW3) == LOW) &&
         (millis() - releaseWait < 500)) {
    delay(10);
  }
}

bool displayIsMenuActive() { return s_inMenu; }
bool displayTestHoldRequested() {
  bool t = s_testHold;
  s_testHold = false;
  return t;
}

void displaySetCalibProgress(uint32_t elapsedMs, uint32_t totalMs) {
  static uint32_t lastDraw = 0;
  uint32_t now = millis();
  if (now - lastDraw < 200 && elapsedMs < totalMs) {
    return;
  }
  lastDraw = now;

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(16, 18, "CALIBRATING...");
  u8g2.drawFrame(10, 26, 108, 12);
  if (totalMs == 0) totalMs = 1;
  int w = (int)((elapsedMs * 104) / totalMs);
  if (w > 104) w = 104;
  if (w > 0) u8g2.drawBox(12, 28, w, 8);
  u8g2.sendBuffer();
}

void displayShowMessage(const char *title, const char *msg, uint32_t durationMs) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  if (title) u8g2.drawStr(8, 25, title);
  if (msg) u8g2.drawStr(8, 45, msg);
  u8g2.sendBuffer();
  if (durationMs > 0) {
    delay(durationMs);
  }
}

void displayShowStatus(const char *mode, int speed, int pos, unsigned char sensorBits) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);

  char header[28];
  snprintf(header, sizeof(header), "%s | Spd:%d", mode, speed);
  u8g2.drawStr(0, 12, header);

  char posBuf[28];
  snprintf(posBuf, sizeof(posBuf), "Pos: %d", pos);
  u8g2.drawStr(0, 26, posBuf);

  for (int i = 0; i < 8; i++) {
    int x = 4 + i * 15;
    u8g2.drawFrame(x, 32, 12, 14);
    if (sensorBits & (0x80 >> i)) {
      u8g2.drawBox(x + 2, 34, 8, 10);
    }
  }

  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(0, 60, "1:Cal 2:Run 3:Fast [H:Menu]");
  u8g2.sendBuffer();
}

void displayShowCalibLive(const unsigned int raw[8], const unsigned int minVal[8], const unsigned int maxVal[8]) {
  u8g2.clearBuffer();

  int okCount = 0;
  for (int i = 0; i < 8; i++) {
    if (maxVal[i] > minVal[i] && (maxVal[i] - minVal[i]) >= 400) {
      okCount++;
    }
  }

  u8g2.setFont(u8g2_font_5x7_tr);
  char header[32];
  if (okCount == 8) {
    snprintf(header, sizeof(header), "DU 8/8 OK! -> Bam SW1 LUU");
  } else {
    snprintf(header, sizeof(header), "Quet qua line: %d/8 mat OK", okCount);
  }
  u8g2.drawStr(0, 8, header);

  const int barTop = 12;
  const int barMaxH = 30;
  for (int i = 0; i < 8; i++) {
    int x = 4 + i * 15;
    u8g2.drawFrame(x, barTop, 12, barMaxH);

    int h = 0;
    if (maxVal[i] > minVal[i] + 50) {
      long v = (long)raw[i] - (long)minVal[i];
      long span = (long)maxVal[i] - (long)minVal[i];
      h = (int)((v * (barMaxH - 4)) / span);
    } else {
      h = (int)((raw[i] * (barMaxH - 4)) / 4095);
    }
    h = constrain(h, 0, barMaxH - 4);
    if (h > 0) {
      u8g2.drawBox(x + 2, barTop + barMaxH - 2 - h, 8, h);
    }

    bool isOk = (maxVal[i] > minVal[i]) && ((maxVal[i] - minVal[i]) >= 400);
    u8g2.drawStr(x + 3, 51, isOk ? "v" : ".");
  }

  u8g2.drawStr(0, 62, "SW1:Luu      SW2:Huy");
  u8g2.sendBuffer();
}
