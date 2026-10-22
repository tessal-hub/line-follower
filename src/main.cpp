// ============================================================
// Line Follower - Anti-Overshoot Reverse Brake + 250Hz PD
// ============================================================
#include <Arduino.h>
#include <EEPROM.h>
#include "config.h"
#include "display.h"

#define EEPROM_SIZE 64
#define BUZZER_PIN  4

// =================== Sensor & PID State ===================
unsigned char isCalib = 0;
int calPID = 0;
unsigned char sensor = 0;               // 8-bit bitmask: bit7 (0x80) = Ch0 (Trái), bit0 (0x01) = Ch7 (Phải)
unsigned int sensorValue[8];
unsigned int sensorPID[8];              // Chuẩn hóa [0..3000] cho PID
unsigned int black_value[8];
unsigned int white_value[8];
unsigned int compare_value[8];          // Ngưỡng phân tách đen/trắng

// =================== PID Output ===================
long lastPos = 0;
long posPID = 0;
float dFiltered = 0.0f;                 // Bộ lọc chống giật cho khâu D (250Hz)
volatile float kp = 15.0f;
volatile float kd = 55.0f;
volatile float kCurve = 5.0f;           // Hệ số tăng lực rẽ cua gắt / góc vuông từ Menu OLED
volatile bool activeBrakeEn = true;     // Bật/tắt phanh chủ động trước cua từ Menu OLED
int servoPwm = 0;                       // Lực lái PID (-4095 .. +4095)

// =================== State Machine & Speed Control ===================
int speed_run_forward = 0;
int nut2 = 2000;                        // Tốc độ vừa (SW2 - Base Speed)
int nut3 = 2800;                        // Tốc độ nhanh (SW3 - Boost Speed)
int nutCorner = 1440;                   // Tốc độ ôm cua (Corner Speed)
bool isHighSpeedMode = false;           // Cờ đánh dấu đang chạy chế độ SW3 (Fast)

int cmdSpeed = 0;                       // Tốc độ tiến vừa cấp cho động cơ (ghi bởi handleAndSpeed)
float vEst = 0.0f;                      // Ước lượng tốc độ thực: tăng tức thì, giảm chậm (quán tính xe)
float currentSpeed = 0.0f;              // Tốc độ thực tế đang cấp (có Ramp & Brake)
int straightCnt = 0;                    // Đếm thời gian chạy thẳng tích lũy động năng (1 đơn vị = 4ms)
int brakeHoldCnt = 0;                   // Đếm số chu kỳ phanh ngược (Reverse Plugging) khi chớm vào cua

int cnt = 0;                            // Đếm thời gian mất line (1 đơn vị = 4ms)
int cnt1 = 0;                           // Đếm timeout khi đang xoay tìm line
unsigned char pattern = 11;
unsigned char start = 0;
unsigned char trangThai = 9;
int RememberLine = 0;                   // +1 (Cua phải), -1 (Cua trái), 0 (Đi thẳng / Nét đứt)
int centerClearCnt = 0;                 // Đếm số chu kỳ đi chuẩn ở giữa để xóa RememberLine

// =================== Non-blocking Button State ===================
static bool lastSw1 = HIGH;
static bool lastSw2 = HIGH;
static bool lastSw3 = HIGH;
static uint32_t sw3PressMs = 0;
static bool sw3HandledLong = false;
static uint32_t lastBtnPollMs = 0;

// =================== Forward Declarations ===================
void speed_run(int speedDC_left, int speedDC_right);
void motors_short_brake();
void handleAndSpeed(int angle, int speed1);
void runforwardline(int tocdo);
void updateLine();
void readEeprom();
void beep(int timerMs);
unsigned char sensorMask(unsigned char mask);
void RGB(int color);
void read_sensor();
void PID();
void pollSensorAndPID();

// =============================================================
// SETUP
// =============================================================
void setup() {
  Serial.begin(115200);

  // Motor pins (DRV8871 H-Bridge: 12-bit PWM 0..4095)
  pinMode(PIN_MOTOR_LEFT_IN1, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_IN2, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_IN2, OUTPUT);
  digitalWrite(PIN_MOTOR_LEFT_IN1, LOW);
  digitalWrite(PIN_MOTOR_LEFT_IN2, LOW);
  digitalWrite(PIN_MOTOR_RIGHT_IN1, LOW);
  digitalWrite(PIN_MOTOR_RIGHT_IN2, LOW);

  ledcSetup(LEDC_CH_LEFT_IN1,  PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(PIN_MOTOR_LEFT_IN1,  LEDC_CH_LEFT_IN1);
  ledcSetup(LEDC_CH_LEFT_IN2,  PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(PIN_MOTOR_LEFT_IN2,  LEDC_CH_LEFT_IN2);
  ledcSetup(LEDC_CH_RIGHT_IN1, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(PIN_MOTOR_RIGHT_IN1, LEDC_CH_RIGHT_IN1);
  ledcSetup(LEDC_CH_RIGHT_IN2, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(PIN_MOTOR_RIGHT_IN2, LEDC_CH_RIGHT_IN2);

  // MUX CD74HC4051 pins
  pinMode(PIN_MUX_S0, OUTPUT);
  pinMode(PIN_MUX_S1, OUTPUT);
  pinMode(PIN_MUX_S2, OUTPUT);
  pinMode(PIN_MUX_SIG, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_MUX_SIG, ADC_11db);

  // Buttons
  pinMode(PIN_BTN_SW1, INPUT_PULLUP);
  pinMode(PIN_BTN_SW2, INPUT_PULLUP);
  pinMode(PIN_BTN_SW3, INPUT_PULLUP);

  // RGB LED
  pinMode(5, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(15, OUTPUT);
  digitalWrite(5, 1);
  digitalWrite(2, 1);
  digitalWrite(15, 1);

  // Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, 0);

  // Trạng thái ban đầu
  speed_run(0, 0);
  trangThai = 9;
  pattern = 11;
  start = 0;
  isCalib = 0;

  EEPROM.begin(EEPROM_SIZE);
  readEeprom();

  displayInit();
  displayShowMessage("LINE FOLLOWER", "READY!", 500);
  beep(40);

  lastSw1 = digitalRead(PIN_BTN_SW1);
  lastSw2 = digitalRead(PIN_BTN_SW2);
  lastSw3 = digitalRead(PIN_BTN_SW3);
}

// =============================================================
// ĐỊNH THỜI ĐỌC SENSOR & TÍNH PID ĐỒNG BỘ 250Hz (4ms / chu kỳ)
// =============================================================
void pollSensorAndPID() {
  static uint32_t lastScanUs = 0;
  uint32_t nowUs = micros();
  if (nowUs - lastScanUs >= 4000) { // 4000 us = 4 ms (250 Hz)
    lastScanUs = nowUs;
    cnt++;
    cnt1++;
    read_sensor();
  }
  PID();
}

// =============================================================
// ĐỌC SENSOR QUA MUX CD74HC4051
// =============================================================
void read_sensor() {
  unsigned char temp = 0;

  for (int j = 0; j < 8; j++) {
    digitalWrite(PIN_MUX_S0, (j & 0x01) ? HIGH : LOW);
    digitalWrite(PIN_MUX_S1, (j & 0x02) ? HIGH : LOW);
    digitalWrite(PIN_MUX_S2, (j & 0x04) ? HIGH : LOW);
    delayMicroseconds(20);
    int r1 = analogRead(PIN_MUX_SIG);
    int r2 = analogRead(PIN_MUX_SIG);
    sensorValue[j] = 4095 - ((r1 + r2) >> 1);
  }

  for (int j = 0; j < 8; j++) {
    if (isCalib == 0) {
      unsigned int b = black_value[j];
      unsigned int w = white_value[j];
      if (w <= b) w = b + 1;

      unsigned int v = sensorValue[j];
      if (v < b) v = b;
      if (v > w) v = w;
      unsigned int mapped = map(v, b, w, 0, 3000);
      sensorPID[j] = (mapped < 200) ? 0 : mapped;
    }
    temp = temp << 1;
    if (sensorValue[j] > compare_value[j]) {
      temp |= 0x01;
    } else {
      temp &= 0xfe;
    }
  }
  sensor = temp;

  // -----------------------------------------------------------------
  // NHẬN DIỆN HƯỚNG RẼ GÓC VUÔNG / CUA GẮT (An toàn tuyệt đối cho Nét đứt):
  // -----------------------------------------------------------------
  // 1. Khi chạm 2 mắt ngoài cùng bên phải (Ch6, Ch7: 0b00000011)
  if ((sensor & 0b00000011) != 0 && (sensor & 0b11000000) == 0) {
    RememberLine = 1;
    centerClearCnt = 0;
  }
  // 2. Khi chạm 2 mắt ngoài cùng bên trái (Ch0, Ch1: 0b11000000)
  else if ((sensor & 0b11000000) != 0 && (sensor & 0b00000011) == 0) {
    RememberLine = -1;
    centerClearCnt = 0;
  }
  // 3. Khi vạch nằm gọn trong 4 mắt giữa (Ch2..Ch5: 0b00111100) và không chạm 2 mắt rìa
  // liên tục 5 chu kỳ (20ms) -> Xóa RememberLine = 0 để sẵn sàng vượt nét đứt
  else if ((sensor & 0b00111100) != 0 && (sensor & 0b11000011) == 0) {
    centerClearCnt++;
    if (centerClearCnt >= 5) {
      RememberLine = 0;
    }
  }

  calPID = 1;
}

// =============================================================
// PID 250Hz + K-CURVE MẠNH CHỐNG VĂNG CUA TỐC ĐỘ CAO
// =============================================================
void PID() {
  if (calPID == 1) {
    calPID = 0;

    // Ước lượng tốc độ thực của xe: tăng theo lệnh ngay, giảm chậm theo quán tính.
    // Gain scheduling dùng vEst (không dùng tốc độ lệnh) để KHÔNG hạ Kd đúng lúc vào cua.
    float cmdF = (float)max(cmdSpeed, 0);
    if (cmdF >= vEst) vEst = cmdF;
    else vEst += (cmdF - vEst) * gParam[P_VEST_DCY];

    if (sensor == 0b00000000) {
      dFiltered = 0.0f;
      return;
    }

    long sum = 0;
    long avg = 0;

    for (int j = 0; j < 8; j++) {
      avg += (long)sensorPID[j] * (j * 3000L);
      sum += (long)sensorPID[j];
    }

    long i = lastPos;
    if (sum > 400) {
      i = (avg / sum) - 10500; // Tâm = 0, Trái < 0 (-10500), Phải > 0 (+10500)
    }
    posPID = i;

    // Vùng chết nhỏ quanh tâm (Deadband ±120) giúp xe đi thẳng đầm
    long err = (abs(i) < 120) ? 0 : i;

    // Tần số 250Hz (4ms) nhanh gấp đôi 125Hz (8ms) nên nhân 2.0f để giữ nguyên thang Kd
    float rawDelta = (float)(i - lastPos) * 2.0f;
    dFiltered = 0.55f * rawDelta + 0.45f * dFiltered;

    // Gain scheduling theo tốc độ (tất cả chỉnh từ Menu OLED):
    // tốc độ cao -> Kp giảm nhẹ, Kd tăng để chống overshoot.
    float refSp = gParam[P_REF_SPD] * 16.0f;
    if (refSp < 640.0f) refSp = 640.0f;
    float vNow = vEst;
    float r = constrain(vNow / refSp, 0.8f, 2.0f);

    float activeKp     = kp * (1.0f - gParam[P_KP_SLOPE] * (r - 1.0f));
    float activeKd     = kd * (1.0f + gParam[P_KD_SLOPE] * (r - 1.0f));
    float activeKCurve = kCurve;
    if (activeKp < 0.0f) activeKp = 0.0f;
    if (isHighSpeedMode) {
      activeKp     *= gParam[P_SW3_KP];
      activeKd     *= gParam[P_SW3_KD];
      activeKCurve *= gParam[P_SW3_KC];
    }

    // Khâu P + D cơ bản
    float pTerm = activeKp * (float)err;
    float dTerm = activeKd * dFiltered * 3.0f;

    // Khâu K-Curve: Kích hoạt sớm từ ngưỡng |err| > 2200 (vừa chớm chạm mắt Ch2 hoặc Ch5)
    // để bánh trong giảm tốc/đảo chiều sớm, chống văng đầu xe ra mép ngoài
    float curveTerm = 0.0f;
    long curveThreshold = (long)gParam[P_CURVE_THR];
    if (abs(err) > curveThreshold && activeKCurve > 0.0f) {
      float excess = (float)(abs(err) - curveThreshold);
      float boost = activeKCurve * (excess * excess) / 2800.0f;
      curveTerm = (err > 0) ? boost : -boost;
    }

    long iRet = (long)((pTerm + dTerm + curveTerm) / 40.0f);

    servoPwm = constrain((int)iRet, -4095, 4095);
    lastPos = i;
  }
}

// =============================================================
// MAIN LOOP
// =============================================================
void loop() {
  pollSensorAndPID();

  bool sw1Click = false;
  bool sw2Click = false;
  bool sw3ShortClick = false;
  bool sw3LongPress = false;

  uint32_t nowMs = millis();
  if (nowMs - lastBtnPollMs >= 20) {
    lastBtnPollMs = nowMs;
    bool cur1 = digitalRead(PIN_BTN_SW1);
    bool cur2 = digitalRead(PIN_BTN_SW2);
    bool cur3 = digitalRead(PIN_BTN_SW3);

    if (cur1 == LOW && lastSw1 == HIGH) sw1Click = true;
    if (cur2 == LOW && lastSw2 == HIGH) sw2Click = true;

    if (cur3 == LOW && lastSw3 == HIGH) {
      sw3PressMs = nowMs;
      sw3HandledLong = false;
    } else if (cur3 == LOW && !sw3HandledLong && (nowMs - sw3PressMs > 600)) {
      sw3HandledLong = true;
      sw3LongPress = true;
    } else if (cur3 == HIGH && lastSw3 == LOW) {
      if (!sw3HandledLong && (nowMs - sw3PressMs >= 20)) {
        sw3ShortClick = true;
      }
    }

    lastSw1 = cur1;
    lastSw2 = cur2;
    lastSw3 = cur3;
  }

  // -----------------------------------------------------------
  // 1. CHẾ ĐỘ CHỜ / IDLE (start == 0)
  // -----------------------------------------------------------
  if (start == 0) {
    if (isCalib == 1) {
      updateLine();

      if (sw2Click) {
        beep(60);
        readEeprom();
        isCalib = 0;
        RGB(99);
        displayShowMessage("CALIBRATION", "DA HUY!", 400);
      } else if (sw1Click) {
        int okSensors = 0;
        for (int i = 0; i < 8; i++) {
          if (white_value[i] > black_value[i] + 200) {
            okSensors++;
          } else {
            black_value[i] = 200;
            white_value[i] = 3500;
          }
          compare_value[i] = (black_value[i] + white_value[i]) / 2;

          EEPROM.write(2 * i,          (black_value[i] >> 8) & 0xFF);
          EEPROM.write(2 * i + 1,      black_value[i] & 0xFF);
          EEPROM.write(16 + 2 * i,     (white_value[i] >> 8) & 0xFF);
          EEPROM.write(17 + 2 * i,     white_value[i] & 0xFF);
        }
        EEPROM.write(32, 0xA5);
        EEPROM.commit();
        RGB(99);
        beep(100);
        isCalib = 0;

        char msg[24];
        snprintf(msg, sizeof(msg), "DA LUU (%d/8 MAT OK)", okSensors);
        displayShowMessage("CALIB SAVED!", msg, 600);
      }
      return;
    }

    static uint32_t lastIdleDispMs = 0;
    if (nowMs - lastIdleDispMs >= 250) {
      lastIdleDispMs = nowMs;
      displayShowStatus("IDLE", nut2 / 16, (int)posPID, sensor);
    }

    if (sw1Click) {
      for (int i = 0; i < 8; i++) {
        black_value[i] = 4095;
        white_value[i] = 0;
        compare_value[i] = 2048;
      }
      beep(80);
      isCalib = 1;
      return;
    }

    if (sw2Click) {
      beep(80);
      displayShowMessage("RUNNING", "SPEED: BASE (SW2)", 0);
      start = 1;
      trangThai = 11;
      pattern = 11;
      isHighSpeedMode = false;
      RememberLine = 0;
      centerClearCnt = 0;
      lastPos = 0;
      dFiltered = 0.0f;
      cnt = 0;
      cnt1 = 0;
      straightCnt = 0;
      brakeHoldCnt = 0;
      speed_run_forward = nut2;
      currentSpeed = (float)nut2;
      return;
    }

    if (sw3ShortClick) {
      beep(80);
      displayShowMessage("RUNNING", "SPEED: FAST (SW3)", 0);
      start = 1;
      trangThai = 11;
      pattern = 11;
      isHighSpeedMode = true;
      RememberLine = 0;
      centerClearCnt = 0;
      lastPos = 0;
      dFiltered = 0.0f;
      cnt = 0;
      cnt1 = 0;
      straightCnt = 0;
      brakeHoldCnt = 0;
      speed_run_forward = nut3;
      currentSpeed = (float)nut2; // Xuất phát êm từ tốc độ Base
      return;
    }

    if (sw3LongPress) {
      beep(80);
      displayMenuLoop();
      lastSw1 = digitalRead(PIN_BTN_SW1);
      lastSw2 = digitalRead(PIN_BTN_SW2);
      lastSw3 = digitalRead(PIN_BTN_SW3);
      if (displayTestHoldRequested()) {
        displayShowMessage("TEST HOLD", "SW1: STOP", 0);
        trangThai = 91;
        start = 1;
      }
      return;
    }

    return;
  }

  // -----------------------------------------------------------
  // 2. CHẾ ĐỘ ĐANG CHẠY (start == 1)
  // -----------------------------------------------------------
  if (sw1Click) {
    speed_run(0, 0);
    RGB(99);
    beep(150);
    start = 0;
    trangThai = 9;
    pattern = 11;
    return;
  }

  switch (trangThai) {
    case 91: // Test PID tại chỗ
      handleAndSpeed(servoPwm, 0);
      break;

    case 11: // Chạy bám line
      runforwardline(speed_run_forward);
      break;

    case 100: // Dừng khi mất line quá lâu
      speed_run(0, 0);
      RGB(nowMs / 150 % 3);
      break;

    default:
      trangThai = 11;
      break;
  }
}

// =============================================================
// RUNFORWARDLINE: Chống Overshoot Bằng Phanh Ngược Chủ Động (Plugging Brake)
// =============================================================
void runforwardline(int tocdo) {
  static uint32_t lastSpeedStepUs = 0;
  bool step4ms = false;
  uint32_t nowUs = micros();
  if (nowUs - lastSpeedStepUs >= 4000) {
    lastSpeedStepUs = nowUs;
    step4ms = true;
  }

  int extraTurn = (int)(kCurve * 45.0f);
  int pivotFwd  = constrain(3000 + extraTurn, 2400, 4095);
  int pivotRev  = constrain(2200 + extraTurn, 1500, 3800);

  switch (pattern) {
    case 11: // Đang bám line bình thường
      if (sensor == 0b00000000) {
        // =========================================================
        // KHI MẤT VẠCH ĐEN (8 MẮT ĐỀU TRẮNG)
        // =========================================================
        // Ở tốc độ cao SW3, nếu trước khi mất vạch xe đã lệch khá xa tâm (|lastPos| >= 4000),
        // thì chắc chắn xe vừa bị văng khỏi góc cua chứ không phải nét đứt!
        long dashLimit = (long)gParam[P_DASH_LIM];

        if (RememberLine == 0 && abs(lastPos) < dashLimit) {
          // -> VẠCH NÉT ĐỨT: Lao thẳng qua khoảng trắng
          RGB(0);
          if (cnt > 250) { // Quá 1.0 giây không thấy vạch mới -> dừng xe
            trangThai = 100;
            pattern = 100;
          } else {
            int dashSpeed = isHighSpeedMode ? min(tocdo, nut2) : tocdo;
            handleAndSpeed(servoPwm / 4, dashSpeed);
          }
        } else {
          // -> GÓC VUÔNG 90 ĐỘ HOẶC VĂNG CUA: Bẻ xoay trục NGAY LẬP TỨC
          cnt1 = 0;
          straightCnt = 0;
          brakeHoldCnt = 0;
          currentSpeed = (float)min(tocdo, nutCorner);

          if (RememberLine == 1 || (RememberLine == 0 && lastPos > 0)) {
            RememberLine = 1;
            speed_run(pivotFwd, -pivotRev);
            pattern = 21;
          } else {
            RememberLine = -1;
            speed_run(-pivotRev, pivotFwd);
            pattern = 31;
          }
        }
      } else {
        // =========================================================
        // ĐANG CÓ LINE: KIỂM SOÁT QUÁN TÍNH & PHANH NGƯỢC CHỐNG VĂNG
        // =========================================================
        cnt = 0;
        RGB(99);

        // 1. NẾU CHẠY CHẾ ĐỘ SW2 (Tốc độ vừa): Giữ nguyên 100% như cũ
        if (!isHighSpeedMode) {
          int curSpeed = tocdo;
          if (abs(posPID) > 7000) {
            curSpeed = (tocdo * 55) / 100;
          } else if (abs(posPID) > 4500) {
            curSpeed = (tocdo * 75) / 100;
          }
          curSpeed -= (int)(fabsf(dFiltered) * gParam[P_DROP]);
          curSpeed = max(curSpeed, (tocdo * 45) / 100);
          handleAndSpeed(servoPwm, curSpeed);
          break;
        }

        // 2. NẾU CHẠY CHẾ ĐỘ SW3 (Tốc độ cao - Boost):
        long absPos = abs(posPID);
        float absD  = fabsf(dFiltered);

        if (step4ms) {
          if (absPos < 1800 && absD < 300.0f) {
            // Đi thẳng trên tâm line: tích lũy và tăng tốc mượt
            if (straightCnt < 500) straightCnt++;
            if (currentSpeed < (float)tocdo) {
              currentSpeed = min(currentSpeed + gParam[P_ACCEL], (float)tocdo);
            }
          } else if (absPos >= (long)gParam[P_BRK_POS]) {
            // Vào cua thật (line tới Ch2/Ch5 trở ra): phanh nếu đã chạy thẳng đủ lâu
            if (activeBrakeEn && straightCnt >= (int)gParam[P_BRK_STRAIGHT]) {
              brakeHoldCnt = (int)gParam[P_BRK_TIME];
            }
            straightCnt = 0;
            currentSpeed = min(currentSpeed, (float)nutCorner);
          }
        }

        // Phanh ngược nhẹ (giới hạn lực lái để không xoay xe)
        if (brakeHoldCnt > 0) {
          if (step4ms) brakeHoldCnt--;
          RGB(0);
          if (gParam[P_BRK_MODE] > 0.5f) {
            // SHORT: hãm động năng 2 bánh (IN1=IN2=full), không cấp lái
            cmdSpeed = 0;
            motors_short_brake();
          } else {
            // PLUG: lùi nền + lái giới hạn
            int brkSteer = (int)gParam[P_BRK_STEER];
            handleAndSpeed(constrain(servoPwm, -brkSteer, brkSteer), -(int)gParam[P_BRK_BASE]);
          }
          break;
        }

        // Giảm tốc liên tục từ lệch 1600 -> 4500 về nutCorner (không nấc)
        int curSpeed = (int)currentSpeed;
        if (absPos > 1600) {
          float k = constrain((absPos - 1600) / 2900.0f, 0.0f, 1.0f);
          curSpeed = (int)((float)curSpeed + ((float)nutCorner - (float)curSpeed) * k);
        }
        // Mép ngoài: giảm sâu nhưng không về 0
        if (absPos > 6000) curSpeed = (nutCorner * 60) / 100;

        // Line trượt càng nhanh (|dFiltered| lớn) càng giảm tốc sớm
        curSpeed -= (int)(absD * gParam[P_DROP]);
        curSpeed = constrain(curSpeed, nutCorner / 2, 4095);
        handleAndSpeed(servoPwm, curSpeed);
      }
      break;

    case 21: // Xoay phải gắt tại chỗ (Pivot Turn) bắt góc vuông 90°
      RGB(1);
      if (cnt1 > 300) { // Quá 1.2 giây không thấy line -> Dừng an toàn
        trangThai = 100;
        pattern = 100;
        break;
      }
      speed_run(pivotFwd, -pivotRev);

      // Đợi cho đến khi vạch đen đi vào các mắt gần tâm (Ch2..Ch5: 0b00111100) mới thoát xoay
      if (sensorMask(0b00111100) != 0) {
        lastPos = 2200;
        dFiltered = 0.0f;
        RememberLine = 0;
        centerClearCnt = 0;
        straightCnt = 0;
        brakeHoldCnt = 0;
        currentSpeed = (float)nutCorner; // Thoát góc vuông ở tốc độ Corner để không văng sang phía bên kia
        pattern = 11;
        cnt = 0;
      }
      break;

    case 31: // Xoay trái gắt tại chỗ (Pivot Turn) bắt góc vuông 90°
      RGB(2);
      if (cnt1 > 300) { // Quá 1.2 giây không thấy line -> Dừng an toàn
        trangThai = 100;
        pattern = 100;
        break;
      }
      speed_run(-pivotRev, pivotFwd);

      // Đợi cho đến khi vạch đen đi vào các mắt gần tâm (Ch2..Ch5: 0b00111100) mới thoát xoay
      if (sensorMask(0b00111100) != 0) {
        lastPos = -2200;
        dFiltered = 0.0f;
        RememberLine = 0;
        centerClearCnt = 0;
        straightCnt = 0;
        brakeHoldCnt = 0;
        currentSpeed = (float)nutCorner; // Thoát góc vuông ở tốc độ Corner để không văng sang phía bên kia
        pattern = 11;
        cnt = 0;
      }
      break;

    case 100:
      speed_run(0, 0);
      break;

    default:
      pattern = 11;
      break;
  }
}

// =============================================================
// CẬP NHẬT CALIBRATION
// =============================================================
void updateLine() {
  static bool beepedAllOk = false;
  int okCount = 0;

  for (int i = 0; i < 8; i++) {
    if (sensorValue[i] < black_value[i]) black_value[i] = sensorValue[i];
    if (sensorValue[i] > white_value[i]) white_value[i] = sensorValue[i];
    compare_value[i] = (black_value[i] + white_value[i]) / 2;
    if (white_value[i] > black_value[i] && (white_value[i] - black_value[i]) >= 400) {
      okCount++;
    }
  }

  if (okCount == 8 && !beepedAllOk) {
    beepedAllOk = true;
    beep(50);
  } else if (okCount < 4) {
    beepedAllOk = false;
  }

  static uint32_t lastCalibDisp = 0;
  if (millis() - lastCalibDisp >= 200) {
    lastCalibDisp = millis();
    displayShowCalibLive(sensorValue, black_value, white_value);
  }
  RGB(okCount == 8 ? 1 : 0);
}

// =============================================================
// ĐỌC EEPROM
// =============================================================
void readEeprom() {
  if (EEPROM.read(32) != 0xA5) {
    for (int i = 0; i < 8; i++) {
      black_value[i]   = 200;
      white_value[i]   = 3500;
      compare_value[i] = 1850;
    }
    return;
  }

  for (int i = 0; i < 8; i++) {
    black_value[i] = ((unsigned int)EEPROM.read(2 * i) << 8) | EEPROM.read(2 * i + 1);
    white_value[i] = ((unsigned int)EEPROM.read(16 + 2 * i) << 8) | EEPROM.read(17 + 2 * i);

    if (white_value[i] <= black_value[i] + 50 || white_value[i] > 4095) {
      black_value[i] = 200;
      white_value[i] = 3500;
    }
    compare_value[i] = black_value[i] + ((white_value[i] - black_value[i]) * 55) / 100;
  }
}

// =============================================================
// MOTOR DRIVER (DRV8871 H-Bridge 4 kênh PWM, 12-bit: -4095..4095)
// =============================================================
void motors_short_brake() {
  ledcWrite(LEDC_CH_LEFT_IN1,  4095);
  ledcWrite(LEDC_CH_LEFT_IN2,  4095);
  ledcWrite(LEDC_CH_RIGHT_IN1, 4095);
  ledcWrite(LEDC_CH_RIGHT_IN2, 4095);
}

void handleAndSpeed(int angle, int speed1) {
  cmdSpeed = speed1;
  int speedLeft  = speed1 + angle;
  int speedRight = speed1 - angle;

  if (speedLeft > 4095) {
    speedRight -= (speedLeft - 4095);
    speedLeft = 4095;
  } else if (speedRight > 4095) {
    speedLeft -= (speedRight - 4095);
    speedRight = 4095;
  }

  speed_run(speedLeft, speedRight);
}

void speed_run(int speedDC_left, int speedDC_right) {
  #if INVERT_LEFT_MOTOR
  speedDC_left = -speedDC_left;
  #endif
  #if INVERT_RIGHT_MOTOR
  speedDC_right = -speedDC_right;
  #endif

  speedDC_left  = constrain(speedDC_left,  -4095, 4095);
  speedDC_right = constrain(speedDC_right, -4095, 4095);

  if (speedDC_left < 0) {
    ledcWrite(LEDC_CH_LEFT_IN1, 0);
    ledcWrite(LEDC_CH_LEFT_IN2, -speedDC_left);
  } else {
    ledcWrite(LEDC_CH_LEFT_IN1, speedDC_left);
    ledcWrite(LEDC_CH_LEFT_IN2, 0);
  }

  if (speedDC_right < 0) {
    ledcWrite(LEDC_CH_RIGHT_IN1, 0);
    ledcWrite(LEDC_CH_RIGHT_IN2, -speedDC_right);
  } else {
    ledcWrite(LEDC_CH_RIGHT_IN1, speedDC_right);
    ledcWrite(LEDC_CH_RIGHT_IN2, 0);
  }
}

// =============================================================
// UTILITIES
// =============================================================
void beep(int timerMs) {
  digitalWrite(BUZZER_PIN, 1);
  delay(timerMs);
  digitalWrite(BUZZER_PIN, 0);
}

unsigned char sensorMask(unsigned char mask) {
  return (sensor & mask);
}

void RGB(int color) {
  switch (color) {
    case 0:
      digitalWrite(5, 0); digitalWrite(2, 1); digitalWrite(15, 1);
      break;
    case 1:
      digitalWrite(5, 1); digitalWrite(2, 0); digitalWrite(15, 1);
      break;
    case 2:
      digitalWrite(5, 1); digitalWrite(2, 1); digitalWrite(15, 0);
      break;
    default:
      digitalWrite(5, 1); digitalWrite(2, 1); digitalWrite(15, 1);
      break;
  }
}