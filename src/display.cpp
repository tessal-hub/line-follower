#include "display.h"
#include "config.h"
#include <Wire.h>
#include <U8g2lib.h>
#include <Preferences.h>

#if OLED_DRIVER_SH1106
static U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ PIN_I2C_SCL, /* data=*/ PIN_I2C_SDA);
#else
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ PIN_I2C_SCL, /* data=*/ PIN_I2C_SDA);
#endif

extern volatile float kp;
extern volatile float kd;
extern volatile float kCurve;
extern volatile bool activeBrakeEn;
extern int nut2;
extern int nut3;
extern int nutCorner;

// Giá trị hiện hành (main.cpp đọc trực tiếp)
float gParam[P_COUNT];

// ------------------------------------------------------------
// Bảng định nghĩa tham số: tên, mặc định, min, max, bước, số lẻ (255 = ON/OFF)
// ------------------------------------------------------------
#define BOOL_DEC 255
#define MODE_DEC 254   // 2 trạng thái: PLUG / SHORT
struct ParamDef {
  const char *name;
  float def, mn, mx, step;
  uint8_t dec;
};

static const ParamDef PDEF[P_COUNT] = {
  {"Bst Spd",  175.0f,   50.0f, 255.0f,   5.0f, 0},   // P_BOOST
  {"Base Spd", 125.0f,   40.0f, 255.0f,   5.0f, 0},   // P_BASE
  {"Cnr Spd",   90.0f,   30.0f, 255.0f,   5.0f, 0},   // P_CORNER
  {"Kp Gain",   15.0f,    0.0f,  50.0f,   0.5f, 1},   // P_KP
  {"Kd Gain",   55.0f,    0.0f, 150.0f,   2.0f, 0},   // P_KD
  {"K-Curve",    5.0f,    0.0f,  30.0f,   1.0f, 1},   // P_KCURVE
  {"ABS Brk",    1.0f,    0.0f,   1.0f,   1.0f, BOOL_DEC}, // P_ABS
  {"Ref Spd",  135.0f,   40.0f, 255.0f,   5.0f, 0},   // P_REF_SPD
  {"Kp Slope",   0.25f,   0.0f,   1.0f,   0.05f, 2},  // P_KP_SLOPE
  {"Kd Slope",   0.60f,   0.0f,   3.0f,   0.10f, 2},  // P_KD_SLOPE
  {"Drop Gn",    0.0f,    0.0f,   5.0f,   0.5f, 1},   // P_DROP
  {"Accel",     25.0f,    1.0f, 100.0f,   5.0f, 0},   // P_ACCEL
  {"Crv Thr", 3200.0f, 1000.0f, 8000.0f, 100.0f, 0},  // P_CURVE_THR
  {"Dash Lim",5500.0f, 2000.0f,10000.0f, 250.0f, 0},  // P_DASH_LIM
  {"Brk Strt",  50.0f,    0.0f, 300.0f,   5.0f, 0},   // P_BRK_STRAIGHT
  {"Brk Pos", 3500.0f, 1500.0f, 8000.0f, 100.0f, 0},  // P_BRK_POS
  {"Brk Base", 600.0f,    0.0f, 2000.0f, 100.0f, 0},  // P_BRK_BASE
  {"Brk Strg",1500.0f,    0.0f, 4000.0f, 100.0f, 0},  // P_BRK_STEER
  {"SW3 Kp",     1.0f,    0.5f,   2.0f,   0.05f, 2},  // P_SW3_KP
  {"SW3 Kd",     1.0f,    0.5f,   2.5f,   0.05f, 2},  // P_SW3_KD
  {"SW3 Kc",     1.0f,    0.5f,   3.0f,   0.10f, 1},  // P_SW3_KC
  {"Brk Time",  10.0f,    3.0f,  40.0f,   1.0f, 0},   // P_BRK_TIME
  {"Brk Mode",   1.0f,    0.0f,   1.0f,   1.0f, MODE_DEC}, // P_BRK_MODE
  {"Vest Dcy",   0.02f,   0.005f, 0.10f,  0.005f, 3}, // P_VEST_DCY
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

static float mCfg[P_COUNT];   // bản đang chỉnh trong menu
static bool s_inMenu = false;
static int  s_menuIndex = 0;
static bool s_menuEdit = false;
static bool s_testHold = false;

static void setDefaults() {
  for (int i = 0; i < P_COUNT; i++) mCfg[i] = PDEF[i].def;
}

// Đẩy mCfg -> gParam và các biến cũ của main.cpp
static void syncToMain() {
  for (int i = 0; i < P_COUNT; i++) gParam[i] = mCfg[i];
  kp = gParam[P_KP];
  kd = gParam[P_KD];
  kCurve = gParam[P_KCURVE];
  activeBrakeEn = (gParam[P_ABS] > 0.5f);
  nut2      = constrain((int)(gParam[P_BASE]   * 16.0f), 800, 4095);
  nut3      = constrain((int)(gParam[P_BOOST]  * 16.0f), 1000, 4095);
  nutCorner = constrain((int)(gParam[P_CORNER] * 16.0f), 600, 4095);
}

static bool isToggle(int idx) {
  return PDEF[idx].dec == BOOL_DEC || PDEF[idx].dec == MODE_DEC;
}

static void stepParam(int idx, int dir) {
  const ParamDef &d = PDEF[idx];
  if (isToggle(idx)) {
    mCfg[idx] = (mCfg[idx] > 0.5f) ? 0.0f : 1.0f;
    return;
  }
  float v = mCfg[idx] + dir * d.step;
  v = constrain(v, d.mn, d.mx);
  v = roundf(v / d.step) * d.step;      // chống trôi số thực
  mCfg[idx] = constrain(v, d.mn, d.mx);
}

static void formatValue(int idx, char *out, size_t n) {
  const ParamDef &d = PDEF[idx];
  if (d.dec == BOOL_DEC) snprintf(out, n, "%s", mCfg[idx] > 0.5f ? "ON" : "OFF");
  else if (d.dec == MODE_DEC) snprintf(out, n, "%s", mCfg[idx] > 0.5f ? "SHORT" : "PLUG");
  else snprintf(out, n, "%.*f", (int)d.dec, mCfg[idx]);
}

static void loadPrefs() {
  setDefaults();
  Preferences p;
  if (p.begin("menu2", true)) {
    if (p.isKey("ver")) {
      char key[8];
      for (int i = 0; i < P_COUNT; i++) {
        snprintf(key, sizeof(key), "v%d", i);
        float v = p.getFloat(key, PDEF[i].def);
        mCfg[i] = constrain(v, PDEF[i].mn, PDEF[i].mx);
      }
    }
    p.end();
  }
  syncToMain();
}

static void savePrefs() {
  Preferences p;
  if (!p.begin("menu2", false)) return;
  char key[8];
  for (int i = 0; i < P_COUNT; i++) {
    snprintf(key, sizeof(key), "v%d", i);
    p.putFloat(key, mCfg[i]);
  }
  p.putUChar("ver", 1);
  p.end();
  syncToMain();
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
  const uint32_t REPEAT_DELAY_MS = 400;   // giữ nút > 400ms bắt đầu lặp
  const uint32_t HOLD_FAST_MS    = 1500;  // giữ > 1.5s: bước x5 (khi đang sửa)

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
        if (!isToggle(s_menuIndex) || first1) stepParam(s_menuIndex, +mult1);
      }
    }
    if (fire2) {
      needRedraw = true;
      if (!s_menuEdit) {
        s_menuIndex = (s_menuIndex + 1) % N_ITEMS;
      } else if (s_menuIndex < P_COUNT) {
        if (!isToggle(s_menuIndex) || first2) stepParam(s_menuIndex, -mult2);
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
        syncToMain();
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