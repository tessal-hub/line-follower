// ============================================================
// Line Follower ESP32 - High-Speed Modular Architecture
// Closed-Loop Encoder PD + DRV8871 Slow Decay (16kHz)
// ============================================================
#include <Arduino.h>
#include "config.h"
#include "params.h"
#include "motor.h"
#include "sensor.h"
#include "encoders.h"
#include "control.h"
#include "display.h"

// Biến trạng thái chế độ Calib
static unsigned char isCalib = 0;

// Bộ đếm chẩn đoán chu kỳ
static uint32_t maxLoopUs = 0;

// Trạng thái phím bấm phi chặn (non-blocking)
static bool lastSw1 = HIGH;
static bool lastSw2 = HIGH;
static bool lastSw3 = HIGH;
static uint32_t sw3PressMs = 0;
static bool sw3HandledLong = false;
static uint32_t lastBtnPollMs = 0;

// Khai báo trước hàm tiện ích
void beep(int timerMs);
void RGB(int color);

// =============================================================
// SETUP
// =============================================================
void setup() {
  // 921600 baud: dòng telemetry ~70 ký tự vừa FIFO TX UART -> không chặn
  // vòng lặp 2kHz (ở 115200 cũ mỗi lần printf chặn ~9ms = mù 17mm tại 2m/s!)
  Serial.begin(TELEMETRY_BAUD);

  // RGB LED
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  RGB(99); // Tắt toàn bộ LED

  // Buzzer
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  // Pushbuttons
  pinMode(PIN_BTN_SW1, INPUT_PULLUP);
  pinMode(PIN_BTN_SW2, INPUT_PULLUP);
  pinMode(PIN_BTN_SW3, INPUT_PULLUP);

  // Khởi tạo các module con
  motorsInit();
  sensorsInit();
  encodersInit();
  paramsInit();
  controlInit();

  displayInit();
  displayShowMessage("LINE FOLLOWER", "SYSTEM READY!", 500);
  beep(40);

  lastSw1 = digitalRead(PIN_BTN_SW1);
  lastSw2 = digitalRead(PIN_BTN_SW2);
  lastSw3 = digitalRead(PIN_BTN_SW3);
}

// =============================================================
// BỘ ĐỊNH THỜI MASTER 2kHz (Đồng bộ Sensor, Encoder & Control)
// Chu kỳ 500µs: quét cảm biến + PID + ghi PWM mỗi tick -> giảm một
// nửa độ trễ pipeline sensor->PWM so với bản 1kHz (cho phép nâng
// Kp/Kd và tốc độ vào cua cao hơn mà vẫn không dao động).
// D-term vẫn cập nhật mỗi 2ms để giữ nguyên thang tuning Kd cũ.
// =============================================================
static void pollMasterScheduler() {
  static uint32_t lastScanUs = 0;
  static uint8_t divMs = 0;
  static uint8_t div2ms = 0;
  static uint8_t div4ms = 0;

  uint32_t nowUs = micros();
  uint32_t dtUs = nowUs - lastScanUs;

  if (dtUs >= CONTROL_PERIOD_US) {
    if (lastScanUs != 0 && dtUs > maxLoopUs && dtUs < 50000) {
      maxLoopUs = dtUs;
    }
    // Bù phần dư thời gian để chu kỳ không trôi tích lũy (drift-free)
    lastScanUs = nowUs - (dtUs % CONTROL_PERIOD_US);

    bool tick1ms = (++divMs >= CONTROL_LOOP_HZ / 1000);
    if (tick1ms) divMs = 0;

    bool tick2ms = (++div2ms >= CONTROL_LOOP_HZ / 500);
    if (tick2ms) div2ms = 0;

    bool tick4ms = (++div4ms >= CONTROL_LOOP_HZ / 250);
    if (tick4ms) div4ms = 0;

    if (tick2ms) {
      // Cập nhật vận tốc từ Encoder mỗi 2ms (trước đây 4ms): vEst và các
      // quyết định phanh/pivot phản ánh vận tốc thực nhanh gấp đôi
      encodersUpdateSpeed();
      float rawMmS = 0.5f * (encodersGetLeftSpeedMmS() + encodersGetRightSpeedMmS());
      realSpeedMmS = 0.70f * rawMmS + 0.30f * realSpeedMmS;

      if (!encActive) {
        int32_t totalTicks = abs(encodersGetLeftTicks()) + abs(encodersGetRightTicks());
        if (totalTicks > 20) encActive = true;
      }
    }

    // Đọc cảm biến 8 kênh qua MUX
    read_sensor();

    // Tính toán góc lái PID & K-Curve
    controlCalculatePID(tick2ms);

    // Cập nhật máy trạng thái chạy line & phanh
    if (currentRunState == STATE_RUN ||
        currentRunState == STATE_PIVOT_RIGHT ||
        currentRunState == STATE_PIVOT_LEFT) {
      controlUpdateRunforward(tick1ms, tick4ms);
    }
  }
}

// =============================================================
// MAIN LOOP
// =============================================================
void loop() {
  pollMasterScheduler();

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
      if (!sw3HandledLong) {
        sw3ShortClick = true;
      }
    }

    lastSw1 = cur1;
    lastSw2 = cur2;
    lastSw3 = cur3;
  }

  // -----------------------------------------------------------
  // 1. CHẾ ĐỘ CHỜ / IDLE
  // -----------------------------------------------------------
  if (currentRunState == STATE_IDLE) {
    if (isCalib == 1) {
      updateCalib();

      if (sw2Click) {
        beep(60);
        readCalib();
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
          compare_value[i] = black_value[i] + ((white_value[i] - black_value[i]) * 55U) / 100U;
        }
        saveCalib();
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
      displayShowStatus("IDLE", gParams.basePwm / 16, (int)posPID, sensorBits);
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

    if (sw2Click || sw3ShortClick) {
      bool fastMode = sw3ShortClick;
      beep(80);
      displayShowMessage("RUNNING", fastMode ? "SPEED: FAST (SW3)" : "SPEED: BASE (SW2)", 0);
      controlResetRun(fastMode);
      return;
    }

    if (sw3LongPress) {
      beep(80);
      displayMenuLoop();
      lastSw1 = digitalRead(PIN_BTN_SW1);
      lastSw2 = digitalRead(PIN_BTN_SW2);
      lastSw3 = digitalRead(PIN_BTN_SW3);
      sw3HandledLong = true;

      if (displayTestHoldRequested()) {
        displayShowMessage("TEST HOLD", "SW1: STOP", 0);
        currentRunState = STATE_TEST_HOLD;
      }
      return;
    }

    return;
  }

  // -----------------------------------------------------------
  // 2. CHẾ ĐỘ ĐANG CHẠY / DỪNG BẢO VỆ
  // -----------------------------------------------------------
  // Nút SW1: Dừng khẩn cấp và quay về trạng thái IDLE
  if (sw1Click) {
    controlEmergencyStop();
    RGB(99);
    beep(150);
    displayShowMessage("STOPPED", "SW1 PRESSED", 400);
    return;
  }

  // In Telemetry qua Serial mỗi 200ms (chỉ khi FIFO TX còn chỗ, không chặn 2kHz loop)
  static uint32_t lastTelemMs = 0;
  if (nowMs - lastTelemMs >= TELEMETRY_PERIOD_MS) {
    lastTelemMs = nowMs;
    Serial.printf("[TELEM] st:%d spd:%.0f vE:%.0f pos:%ld P:%.0f D:%.0f brk:%d/%d wl:%d dt:%uus\n",
                  currentRunState, realSpeedMmS, vEst, posPID, dbgPTerm, dbgDTerm,
                  brakeHoldCnt, pivotBrakeTicks, gParams.wheelLoopEn ? 1 : 0, maxLoopUs);
    maxLoopUs = 0;
  }

  switch (currentRunState) {
    case STATE_TEST_HOLD:
      handleAndSpeed(servoPwm, 0);
      break;

    case STATE_RUN: {
      // Chỉ ghi LED khi đổi trạng thái phanh (tránh ghi GPIO 2000 lần/s)
      static int lastRunRgb = -1;
      int runRgb = (brakeHoldCnt > 0) ? 0 : 99;
      if (runRgb != lastRunRgb) {
        RGB(runRgb);
        lastRunRgb = runRgb;
      }
      break;
    }

    case STATE_PIVOT_RIGHT:
      RGB(1);
      break;

    case STATE_PIVOT_LEFT:
      RGB(2);
      break;

    case STATE_SAFE_STOP:
      // Dừng an toàn khi mất vạch quá lâu:
      // Bánh xe ngắt hoàn toàn; KHÔNG tự động phục hồi khi thấy line giữa (chống tự phóng khi nhấc xe)
      // Người dùng bắt buộc phải nhấn SW1 để xác nhận về IDLE!
      motors_coast();
      RGB((nowMs / 200) % 3);
      break;

    case STATE_FINISH_STOP:
      // Dừng tại vạch đích
      motors_coast();
      RGB(1); // Xanh lá
      beep(30);
      displayShowMessage("FINISH LINE!", "PRESS SW1 TO RESET", 0);
      while (digitalRead(PIN_BTN_SW1) == HIGH) {
        delay(20);
      }
      controlEmergencyStop();
      beep(60);
      break;

    default:
      controlEmergencyStop();
      break;
  }
}

// =============================================================
// TIỆN ÍCH
// =============================================================
void beep(int timerMs) {
  digitalWrite(PIN_BUZZER, HIGH);
  delay(timerMs);
  digitalWrite(PIN_BUZZER, LOW);
}

void RGB(int color) {
  switch (color) {
    case 0: // Đỏ
      digitalWrite(PIN_LED_R, LOW);
      digitalWrite(PIN_LED_G, HIGH);
      digitalWrite(PIN_LED_B, HIGH);
      break;
    case 1: // Xanh lá
      digitalWrite(PIN_LED_R, HIGH);
      digitalWrite(PIN_LED_G, LOW);
      digitalWrite(PIN_LED_B, HIGH);
      break;
    case 2: // Xanh dương
      digitalWrite(PIN_LED_R, HIGH);
      digitalWrite(PIN_LED_G, HIGH);
      digitalWrite(PIN_LED_B, LOW);
      break;
    default: // Tắt hết
      digitalWrite(PIN_LED_R, HIGH);
      digitalWrite(PIN_LED_G, HIGH);
      digitalWrite(PIN_LED_B, HIGH);
      break;
  }
}
