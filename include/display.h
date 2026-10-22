#pragma once
#include <Arduino.h>

// ============================================================
// Danh sách tham số chỉnh được từ Menu OLED (giữ sw3 để vào menu)
// Giá trị hiện hành nằm trong mảng gParam[] (main.cpp đọc trực tiếp).
// ============================================================
enum ParamId {
  P_BOOST = 0,    // Tốc độ SW3 (0..255, x16 -> PWM 12-bit)
  P_BASE,         // Tốc độ SW2
  P_CORNER,       // Tốc độ ôm cua
  P_KP,           // Kp
  P_KD,           // Kd
  P_KCURVE,       // K-Curve
  P_ABS,          // Phanh chủ động ON/OFF
  P_REF_SPD,      // Tốc độ tham chiếu (đang chạy ổn), gain scheduling
  P_KP_SLOPE,     // Kp giảm bao nhiêu khi tốc độ tăng gấp đôi ref
  P_KD_SLOPE,     // Kd tăng bao nhiêu khi tốc độ tăng gấp đôi ref
  P_DROP,         // Hệ số giảm tốc theo |dFiltered| (0 = tắt)
  P_ACCEL,        // Bước tăng tốc mỗi 4ms (SW3)
  P_CURVE_THR,    // Ngưỡng K-Curve bắt đầu tác dụng
  P_DASH_LIM,     // |lastPos| tối đa để coi là nét đứt
  P_BRK_STRAIGHT, // Số chu kỳ 4ms chạy thẳng tối thiểu trước khi phanh
  P_BRK_POS,      // |posPID| kích hoạt phanh (vào cua thật)
  P_BRK_BASE,     // Lực lùi nền khi phanh (độ lớn, áp dụng dấu âm)
  P_BRK_STEER,    // Giới hạn lực lái khi phanh
  P_SW3_KP,       // Hệ số nhân Kp riêng SW3
  P_SW3_KD,       // Hệ số nhân Kd riêng SW3
  P_SW3_KC,       // Hệ số nhân K-Curve riêng SW3
  P_BRK_TIME,     // Thời gian phanh (số chu kỳ 4ms)
  P_BRK_MODE,     // 0 = PLUG (lùi bánh), 1 = SHORT (hãm động năng)
  P_VEST_DCY,     // Tốc độ "quên" của ước lượng tốc độ thực (nhỏ = nhớ lâu)
  P_COUNT
};

extern float gParam[P_COUNT];

void displayInit();
void displayMenuLoop();
bool displayIsMenuActive();
bool displayTestHoldRequested();
void displaySetCalibProgress(uint32_t elapsedMs, uint32_t totalMs);
void displayShowMessage(const char *title, const char *msg, uint32_t durationMs = 1500);
void displayShowStatus(const char *mode, int speed, int pos, unsigned char sensorBits);
void displayShowCalibLive(const unsigned int raw[8], const unsigned int minVal[8], const unsigned int maxVal[8]);