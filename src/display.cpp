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
extern volatile int kd;
extern volatile float kCurve;
extern int nut2;
extern int nut3;

struct MenuConfig {
  float boostSpeed = 175.0f;
  float baseSpeed = 125.0f;
  float cornerSpeed = 90.0f;
  float kpGain = 15.0f;
  float kdGain = 55.0f;
  float kCurveGain = 5.0f;
  bool  activeBrakeEn = true;
};

static MenuConfig mCfg;
static bool s_inMenu = false;
static int  s_menuIndex = 0;
static bool s_menuEdit = false;
static bool s_testHold = false;

static const char *MENU_NAME[11] = {
  "Bst Spd","Base Spd","Cnr Spd","Kp Gain","Kd Gain","K-Curve","ABS Brk",
  "[SAVE & EXIT]","[CANCEL]","[RESTORE DEF]","[TEST HOLD]"
};
static float MENU_STEP[11] = {5,5,5,0.5f,2.0f,1.0f,0,0,0,0,0};
static float MENU_MIN[11]  = {50,40,30,0,0,0,0,0,0,0,0};
static float MENU_MAX[11]  = {255,255,255,50,150,30,0,0,0,0,0};

static void syncToMain() {
  kp = mCfg.kpGain;
  kd = (int)mCfg.kdGain;
  kCurve = mCfg.kCurveGain;
  nut2 = constrain((int)(mCfg.baseSpeed * 16.0f), 800, 4095);
  nut3 = constrain((int)(mCfg.boostSpeed * 16.0f), 1000, 4095);
}

static float menuValue(int idx) {
  switch (idx) {
    case 0: return mCfg.boostSpeed;
    case 1: return mCfg.baseSpeed;
    case 2: return mCfg.cornerSpeed;
    case 3: return mCfg.kpGain;
    case 4: return mCfg.kdGain;
    case 5: return mCfg.kCurveGain;
    case 6: return mCfg.activeBrakeEn ? 1.0f : 0.0f;
    default: return 0.0f;
  }
}

static void menuSetValue(int idx, float v) {
  switch (idx) {
    case 0: mCfg.boostSpeed = v; break;
    case 1: mCfg.baseSpeed = v; break;
    case 2: mCfg.cornerSpeed = v; break;
    case 3: mCfg.kpGain = v; break;
    case 4: mCfg.kdGain = v; break;
    case 5: mCfg.kCurveGain = v; break;
    case 6: mCfg.activeBrakeEn = (v > 0.5f); break;
  }
}

static void loadPrefs() {
  Preferences p;
  if (p.begin("menu_cfg", true)) {
    if (p.isKey("valid")) {
      mCfg.boostSpeed    = p.getFloat("bst", 175.0f);
      mCfg.baseSpeed     = p.getFloat("base", 125.0f);
      mCfg.cornerSpeed   = p.getFloat("cnr", 90.0f);
      mCfg.kpGain        = p.getFloat("kp", 15.0f);
      mCfg.kdGain        = p.getFloat("kd", 55.0f);
      mCfg.kCurveGain    = p.getFloat("kc", 5.0f);
      mCfg.activeBrakeEn = p.getBool("brk", true);
    }
    p.end();
  }
  syncToMain();
}

static void savePrefs() {
  Preferences p;
  if (!p.begin("menu_cfg", false)) return;
  p.putFloat("bst", mCfg.boostSpeed);
  p.putFloat("base", mCfg.baseSpeed);
  p.putFloat("cnr", mCfg.cornerSpeed);
  p.putFloat("kp", mCfg.kpGain);
  p.putFloat("kd", mCfg.kdGain);
  p.putFloat("kc", mCfg.kCurveGain);
  p.putBool("brk", mCfg.activeBrakeEn);
  p.putBool("valid", true);
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
  s_inMenu = true;
  bool last1 = (digitalRead(PIN_BTN_SW1) == LOW);
  bool last2 = (digitalRead(PIN_BTN_SW2) == LOW);
  bool last3 = (digitalRead(PIN_BTN_SW3) == LOW);
  bool needRedraw = true;

  while (s_inMenu) {
    bool sw1 = (digitalRead(PIN_BTN_SW1) == LOW);
    bool sw2 = (digitalRead(PIN_BTN_SW2) == LOW);
    bool sw3 = (digitalRead(PIN_BTN_SW3) == LOW);

    if (sw1 && !last1) {
      needRedraw = true;
      if (!s_menuEdit) {
        s_menuIndex = (s_menuIndex + 10) % 11;
      } else if (s_menuIndex == 6) {
        mCfg.activeBrakeEn = !mCfg.activeBrakeEn;
      } else if (s_menuIndex < 6) {
        float v = menuValue(s_menuIndex) + MENU_STEP[s_menuIndex];
        if (v > MENU_MAX[s_menuIndex]) v = MENU_MAX[s_menuIndex];
        menuSetValue(s_menuIndex, v);
      }
    }
    if (sw2 && !last2) {
      needRedraw = true;
      if (!s_menuEdit) {
        s_menuIndex = (s_menuIndex + 1) % 11;
      } else if (s_menuIndex == 6) {
        mCfg.activeBrakeEn = !mCfg.activeBrakeEn;
      } else if (s_menuIndex < 6) {
        float v = menuValue(s_menuIndex) - MENU_STEP[s_menuIndex];
        if (v < MENU_MIN[s_menuIndex]) v = MENU_MIN[s_menuIndex];
        menuSetValue(s_menuIndex, v);
      }
    }
    if (sw3 && !last3) {
      needRedraw = true;
      if (s_menuEdit) {
        s_menuEdit = false;
      } else if (s_menuIndex < 7) {
        s_menuEdit = true;
      } else if (s_menuIndex == 7) {
        savePrefs();
        s_inMenu = false;
      } else if (s_menuIndex == 8) {
        loadPrefs();
        s_inMenu = false;
      } else if (s_menuIndex == 9) {
        mCfg = MenuConfig();
        savePrefs();
      } else if (s_menuIndex == 10) {
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
      int top = (s_menuIndex >= 3) ? s_menuIndex - 2 : 0;
      if (top > 7) top = 7;
      char buf[48];
      char val[16];
      for (int r = 0; r < 4; r++) {
        int idx = top + r;
        if (idx >= 11) break;
        bool sel = (idx == s_menuIndex);
        val[0] = '\0';
        if (idx < 7) {
          if (idx == 6) snprintf(val, sizeof(val), "%s", mCfg.activeBrakeEn ? "ON" : "OFF");
          else snprintf(val, sizeof(val), "%.1f", menuValue(idx));
          if (s_menuEdit && sel) {
            char t[24];
            snprintf(t, sizeof(t), "<%s>", val);
            snprintf(val, sizeof(val), "%s", t);
          }
        }
        snprintf(buf, sizeof(buf), "%c %s %s", sel ? '>' : ' ', MENU_NAME[idx], val);
        u8g2.drawStr(0, 22 + r * 12, buf);
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