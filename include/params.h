#pragma once
#include <Arduino.h>

// ============================================================
// Danh sách 10 tham số cốt lõi chỉnh từ Menu OLED
// ============================================================
enum ParamId {
  P_BOOST = 0,    // 1. Bst Spd  : Tốc độ tối đa SW3 (PWM: 80..255 * 16)
  P_BASE,         // 2. Base Spd : Tốc độ chuẩn SW2 (PWM: 70..255 * 16)
  P_CORNER,       // 3. Cnr Spd  : Tốc độ ôm cua mục tiêu (PWM: 85..220 * 16)
  P_KP,           // 4. Kp Gain  : Hệ số tỉ lệ lái P
  P_KD,           // 5. Kd Gain  : Hệ số đạo hàm dập lắc D
  P_KCURVE,       // 6. K-Curve  : Hệ số bẻ lái phi tuyến cua gắt & xoay 90°
  P_ACCEL,        // 7. Accel    : Bước gia tốc đường thẳng mỗi 4ms
  P_BRK_PWR,      // 8. Brk Pwr  : Cường độ phanh chủ động (0%=OFF, 5..100%)
  P_BRK_POS,      // 9. Brk Pos  : Ngưỡng vị trí lệch vạch kích hoạt phanh
  P_WLOOP,        // 10. W-Loop  : Vòng lặp kín vận tốc bánh xe (OFF/ON)
  P_COUNT
};

// ============================================================
// Tham số vận hành thực tế đã giải mã ra thang điều khiển
// ============================================================
struct RobotParams {
  int   boostPwm;         // Tốc độ SW3 (1280 .. 4095)
  int   basePwm;          // Tốc độ SW2 (1120 .. 4095)
  int   cornerPwm;        // Tốc độ ôm cua (1360 .. 4095)
  float kp;               // Hệ số Kp
  float kd;               // Hệ số Kd
  float kCurve;           // Hệ số K-Curve
  float accelStep;        // Gia tốc mỗi 4ms (PWM tick)
  float brkPowerPct;      // Phần trăm lực phanh 0..100%
  float brkTriggerPos;    // Ngưỡng kích hoạt phanh (1500..6000)
  bool  activeBrakeEn;    // Cho phép phanh chủ động (brkPowerPct >= 5%)
  bool  wheelLoopEn;      // Vòng lặp kín vận tốc bánh xe (Encoder FF+PI)
};

// ============================================================
// Gom toàn bộ hằng số ẩn (Tuning Magic Numbers) vào struct có tên
// ============================================================
struct TuningConstants {
  // Bộ lọc đạo hàm D và tỉ lệ PID
  float dTermScale;           // 3.0f - Khâu khuếch đại D-Term
  float dFilterAlpha;         // 0.18f - Trọng số EMA cập nhật đạo hàm
  float pidDivider;           // 40.0f - Hệ số quy đổi PID sang xung servoPwm
  long  deadbandPos;          // 120L - Ngưỡng deadband tâm đường thẳng

  // Dự đoán vị trí vạch (Phase-Lead Prediction)
  float leadFactor;           // 0.45f - Dự đoán vạch sớm dựa trên vận tốc trôi ngang
  long  leadMaxOffset;        // 1200L - Giới hạn tối đa bước bù dự đoán

  // Giảm tốc trong cua (Corner Dynamic Slowdown)
  float cornerDecelBase;      // 0.20f - Tỉ lệ hãm cơ bản khi xe hướng ra mép cua
  float cornerDecelBrkScale;  // 0.25f - Tỉ lệ hãm tăng thêm theo Brk Pwr

  // Xoay trục 90° (Pivot Turn)
  float pivotKCurveScale;     // 1.25f - Hệ số nhân K-Curve ở chế độ High Speed
  float pivotTurnCoeff;       // 45.0f - Hệ số sinh lực xoay pivot từ K-Curve
  int   minTorqueFloor;       // 1350 - Sàn mô-men xoắn tối thiểu giữ bánh bám đường

  // Vòng lặp kín vận tốc bánh xe (Wheel Velocity Closed-Loop)
  float wlKp;                 // 0.40f - PWM bổ sung cho mỗi mm/s sai số tốc độ
  float wlKi;                 // 3.0f  - PWM bổ sung cho mỗi (mm/s·s) tích lũy
  float wlIntegMax;           // 600.0f - Kẹp tích phân chống windup
  uint32_t wlStaleUs;         // 4000 - Reset tích phân nếu không được gọi (bị phanh/pivot chiếm)
};

extern RobotParams gParams;
extern const TuningConstants gTuning;
extern float gMenuParams[P_COUNT];

void paramsInit();
void paramsSyncFromMenu();
