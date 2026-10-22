// ============================================================
// Line Follower - Smooth PD + K-Curve + Fast 90-Degree Turn
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
int PIDfre = 0;
unsigned int sensorValue[8];
unsigned int sensorPID[8];              // Chuẩn hóa [0..3000] cho PID
unsigned int black_value[8];
unsigned int white_value[8];
unsigned int compare_value[8];          // Ngưỡng phân tách đen/trắng

// =================== PID Output ===================
long lastPos = 0;
long posPID = 0;
float dFiltered = 0.0f;                 // Bộ lọc chống giật cho khâu D
volatile float kp = 15.0f;
volatile int kd = 55;
volatile float kCurve = 5.0f;           // Hệ số tăng lực rẽ cua gắt / góc vuông từ Menu OLED
int servoPwm = 0;                       // Lực lái PID (-4095 .. +4095)

// =================== State Machine ===================
int speed_run_forward = 0;
int nut2 = 2000;                        // Tốc độ vừa (SW2)
int nut3 = 2800;                        // Tốc độ nhanh (SW3)
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
// ĐỊNH THỜI ĐỌC SENSOR 250Hz BẰNG MICROS() (4ms / chu kỳ)
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
  PIDfre++;

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

  // Nhận diện hướng rẽ góc vuông / cua gắt:
  // 1. Khi có >= 3 mắt lệch hẳn về bên phải (ví dụ 0b00000111, 0b00001111, 0b00011111) hoặc chạm mắt ngoài cùng bên phải (bit 0, 1)
  if ((sensor & 0b00000011) != 0 && (sensor & 0b11000000) == 0) {
    RememberLine = 1;
    centerClearCnt = 0;
  }
  // 2. Khi chạm mắt ngoài cùng bên trái (bit 6, 7)
  else if ((sensor & 0b11000000) != 0 && (sensor & 0b00000011) == 0) {
    RememberLine = -1;
    centerClearCnt = 0;
  }
  // 3. Nếu vạch chỉ nằm gọn ở 4 mắt giữa liên tục ít nhất 6 chu kỳ (24ms) -> Chắc chắn đang đi thẳng (reset RememberLine = 0 để sẵn sàng vượt nét đứt)
  else if ((sensor & 0b00111100) != 0 && (sensor & 0b11000011) == 0) {
    centerClearCnt++;
    if (centerClearCnt >= 6) {
      RememberLine = 0;
    }
  }

  if (PIDfre >= 2) { // Tính PID ở 125Hz (mỗi 8ms)
    PIDfre = 0;
    calPID = 1;
  }
}

// =============================================================
// PID + K-CURVE TĂNG LỰC BẺ GÓC
// =============================================================
void PID() {
  if (calPID == 1) {
    calPID = 0;

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

    // Vùng chết nhỏ quanh tâm (Deadband ±150) giúp xe đi thẳng đầm, không lắc đầu
    long err = (abs(i) < 150) ? 0 : i;

    // Lọc thông thấp (EMA) cho đạo hàm vị trí để triệt tiêu gai nhiễu
    float rawDelta = (float)(i - lastPos);
    dFiltered = 0.55f * rawDelta + 0.45f * dFiltered;

    // Khâu P + D cơ bản (êm ái ở vùng giữa)
    float pTerm = kp * (float)err;
    float dTerm = (float)kd * dFiltered * 3.0f;

    // Khâu K-Curve: Khi xe lệch ra ngoài vùng giữa (|err| > 3500), K-Curve bơm thêm lực bẻ lái mạnh để ép đầu xe vào góc cua
    float curveTerm = 0.0f;
    if (abs(err) > 3500 && kCurve > 0.0f) {
      float excess = (float)(abs(err) - 3500); // 0 .. 7000
      float boost = kCurve * (excess * excess) / 3500.0f;
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
      displayShowStatus("IDLE", nut2, (int)posPID, sensor);
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
      RememberLine = 0;
      centerClearCnt = 0;
      lastPos = 0;
      dFiltered = 0.0f;
      cnt = 0;
      cnt1 = 0;
      speed_run_forward = nut2;
      return;
    }

    if (sw3ShortClick) {
      beep(80);
      displayShowMessage("RUNNING", "SPEED: FAST (SW3)", 0);
      start = 1;
      trangThai = 11;
      pattern = 11;
      RememberLine = 0;
      centerClearCnt = 0;
      lastPos = 0;
      dFiltered = 0.0f;
      cnt = 0;
      cnt1 = 0;
      speed_run_forward = nut3;
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
// RUNFORWARDLINE: Bám line + Vượt Nét Đứt + Xoay Góc Vuông 90° Cực Nhanh
// =============================================================
void runforwardline(int tocdo) {
  // Tính lực xoay góc vuông dựa trên K-Curve (mặc định mạnh: tiến 3100, lùi -2200; tăng K-Curve sẽ tăng thêm lực xoay)
  int extraTurn = (int)(kCurve * 40.0f);
  int pivotFwd  = constrain(2900 + extraTurn, 2400, 4095);
  int pivotRev  = constrain(2000 + extraTurn, 1400, 3600);

  switch (pattern) {
    case 11: // Đang bám line bình thường
      if (sensor == 0b00000000) {
        // --- KHI MẤT VẠCH ĐEN (8 MẮT ĐỀU TRẮNG) ---

        // Trường hợp 1: Đang đi ở vùng giữa (RememberLine == 0 và độ lệch cuối nhỏ)
        // -> VẠCH NÉT ĐỨT: Lao thẳng qua khoảng trắng
        if (RememberLine == 0 && abs(lastPos) < 5500) {
          RGB(0);
          if (cnt > 250) { // Quá 1.0 giây không thấy vạch mới -> dừng xe
            trangThai = 100;
            pattern = 100;
          } else {
            handleAndSpeed(servoPwm / 4, tocdo);
          }
        }
        // Trường hợp 2: Vạch đã quét qua mắt mép ngoài (RememberLine != 0 hoặc lệch lớn)
        // -> GÓC VUÔNG 90 ĐỘ: Bẻ xoay trục NGAY LẬP TỨC (0ms delay) để không bị lao lố!
        else {
          cnt1 = 0;
          if (RememberLine == 1 || lastPos > 0) {
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
        // --- ĐANG CÓ LINE ---
        cnt = 0;
        RGB(99);

        // Nếu chỉ còn 1-2 mắt ngoài cùng dính line (chớm vào góc vuông), giảm tốc tiến xuống 55%
        // để lực bẻ lái (servoPwm) làm bánh trong lùi ngay lập tức, giúp xe xoay cực gắt!
        int curSpeed = tocdo;
        if (abs(posPID) > 7000) {
          curSpeed = (tocdo * 55) / 100;
        } else if (abs(posPID) > 4500) {
          curSpeed = (tocdo * 75) / 100;
        }
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
      // Giúp xe xoay đủ 90 độ, không bị thoát xoay sớm khi mới chạm mép Ch7!
      if (sensorMask(0b00111100) != 0) {
        lastPos = 2500;
        dFiltered = 0.0f;
        RememberLine = 0;
        centerClearCnt = 0;
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
        lastPos = -2500;
        dFiltered = 0.0f;
        RememberLine = 0;
        centerClearCnt = 0;
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
void handleAndSpeed(int angle, int speed1) {
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