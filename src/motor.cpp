#include "motor.h"
#include "config.h"
#include "params.h"
#include "encoders.h"
#include "control.h"

static int s_cmdSpeed = 0;

// ============================================================
// Vòng lặp kín vận tốc bánh xe (Cascade inner-loop):
//   out = Feedforward (PWM lệnh gốc) + Kp*err + Integral
// - Bù sụt áp pin & sai khác động cơ: cùng một lệnh PWM cho cùng
//   vận tốc thực mọi thời điểm, giúp thông số tuning ổn định suốt trận.
// - Tự bypass khi: tắt W-Loop, Encoder chưa active, hoặc bị luồng
//   phanh/pivot chiếm dụng (speed_run thô) quá wlStaleUs.
// ============================================================
static bool  s_wheelLoopEn  = true;
static float s_integL = 0.0f;
static float s_integR = 0.0f;
static uint32_t s_lastClUs = 0;

void motorsSetWheelLoop(bool enabled) {
  if (!enabled) motorsResetWheelLoop();
  s_wheelLoopEn = enabled;
}

void motorsResetWheelLoop() {
  s_integL = 0.0f;
  s_integR = 0.0f;
  s_lastClUs = 0;
}

static inline int wheelClamp(int v) {
  return constrain(v, -MAX_PLUG_PWM, 4095);
}

static void speed_run_closed(int speedDC_left, int speedDC_right) {
  uint32_t nowUs = micros();

  if (!s_wheelLoopEn || !encActive) {
    s_integL = 0.0f;
    s_integR = 0.0f;
    s_lastClUs = nowUs;
    speed_run(speedDC_left, speedDC_right);
    return;
  }

  float dt = 0.0f;
  if (s_lastClUs != 0) {
    dt = (float)(nowUs - s_lastClUs) * 1e-6f;
    // Bị phanh ngược / pivot chiếm điều khiển (gọi speed_run thô) -> reset tích phân
    if (dt > (float)gTuning.wlStaleUs * 1e-6f) {
      s_integL = 0.0f;
      s_integR = 0.0f;
      dt = 0.0f;
    }
  }
  s_lastClUs = nowUs;

  // Chuyển lệnh PWM thang điều khiển sang mục tiêu vận tốc vật lý (mm/s)
  float tgtL = (float)speedDC_left  / PWM_PER_MMS;
  float tgtR = (float)speedDC_right / PWM_PER_MMS;
  float errL = tgtL - encodersGetLeftSpeedMmS();
  float errR = tgtR - encodersGetRightSpeedMmS();

  s_integL = constrain(s_integL + gTuning.wlKi * errL * dt, -gTuning.wlIntegMax, gTuning.wlIntegMax);
  s_integR = constrain(s_integR + gTuning.wlKi * errR * dt, -gTuning.wlIntegMax, gTuning.wlIntegMax);

  int outL = wheelClamp(speedDC_left  + (int)roundf(gTuning.wlKp * errL + s_integL));
  int outR = wheelClamp(speedDC_right + (int)roundf(gTuning.wlKp * errR + s_integR));
  speed_run(outL, outR);
}

static inline void writeMotorPwm(uint8_t pin, uint8_t ch, uint32_t duty) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  (void)ch;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(ch, duty);
#endif
}

void motorsInit() {
  pinMode(PIN_MOTOR_LEFT_IN1, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_IN2, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_IN2, OUTPUT);

  digitalWrite(PIN_MOTOR_LEFT_IN1, LOW);
  digitalWrite(PIN_MOTOR_LEFT_IN2, LOW);
  digitalWrite(PIN_MOTOR_RIGHT_IN1, LOW);
  digitalWrite(PIN_MOTOR_RIGHT_IN2, LOW);

  const uint32_t safePwmFreq = (PWM_FREQ_HZ > 19000) ? 16000 : PWM_FREQ_HZ;

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcAttach(PIN_MOTOR_LEFT_IN1,  safePwmFreq, PWM_RES_BITS);
  ledcAttach(PIN_MOTOR_LEFT_IN2,  safePwmFreq, PWM_RES_BITS);
  ledcAttach(PIN_MOTOR_RIGHT_IN1, safePwmFreq, PWM_RES_BITS);
  ledcAttach(PIN_MOTOR_RIGHT_IN2, safePwmFreq, PWM_RES_BITS);
#else
  ledcSetup(LEDC_CH_LEFT_IN1,  safePwmFreq, PWM_RES_BITS);
  ledcAttachPin(PIN_MOTOR_LEFT_IN1,  LEDC_CH_LEFT_IN1);
  ledcSetup(LEDC_CH_LEFT_IN2,  safePwmFreq, PWM_RES_BITS);
  ledcAttachPin(PIN_MOTOR_LEFT_IN2,  LEDC_CH_LEFT_IN2);
  ledcSetup(LEDC_CH_RIGHT_IN1, safePwmFreq, PWM_RES_BITS);
  ledcAttachPin(PIN_MOTOR_RIGHT_IN1, LEDC_CH_RIGHT_IN1);
  ledcSetup(LEDC_CH_RIGHT_IN2, safePwmFreq, PWM_RES_BITS);
  ledcAttachPin(PIN_MOTOR_RIGHT_IN2, LEDC_CH_RIGHT_IN2);
#endif

  motors_coast();
}

int motorsGetCmdSpeed() {
  return s_cmdSpeed;
}

void motorsSetCmdSpeed(int speed) {
  s_cmdSpeed = speed;
}

void motors_short_brake() {
  s_cmdSpeed = 0;
  motorsResetWheelLoop();
  writeMotorPwm(PIN_MOTOR_LEFT_IN1,  LEDC_CH_LEFT_IN1,  4095);
  writeMotorPwm(PIN_MOTOR_LEFT_IN2,  LEDC_CH_LEFT_IN2,  4095);
  writeMotorPwm(PIN_MOTOR_RIGHT_IN1, LEDC_CH_RIGHT_IN1, 4095);
  writeMotorPwm(PIN_MOTOR_RIGHT_IN2, LEDC_CH_RIGHT_IN2, 4095);
}

void motors_coast() {
  s_cmdSpeed = 0;
  motorsResetWheelLoop();
  writeMotorPwm(PIN_MOTOR_LEFT_IN1,  LEDC_CH_LEFT_IN1,  0);
  writeMotorPwm(PIN_MOTOR_LEFT_IN2,  LEDC_CH_LEFT_IN2,  0);
  writeMotorPwm(PIN_MOTOR_RIGHT_IN1, LEDC_CH_RIGHT_IN1, 0);
  writeMotorPwm(PIN_MOTOR_RIGHT_IN2, LEDC_CH_RIGHT_IN2, 0);
}

void speed_run(int speedDC_left, int speedDC_right) {
#if INVERT_LEFT_MOTOR
  speedDC_left = -speedDC_left;
#endif
#if INVERT_RIGHT_MOTOR
  speedDC_right = -speedDC_right;
#endif

  // Giới hạn an toàn:
  // - Tiến (dương): tối đa 4095 (100% duty)
  // - Lùi / Hãm ngược (âm): kẹp ở -MAX_PLUG_PWM (-3000) chống sụt áp nguồn 12V brownout ESP32!
  speedDC_left  = constrain(speedDC_left,  -MAX_PLUG_PWM, 4095);
  speedDC_right = constrain(speedDC_right, -MAX_PLUG_PWM, 4095);

  // Điều khiển DRV8871 Slow Decay:
  // Tiến: IN1 = HIGH (4095), IN2 = PWM đảo (4095 - duty)
  // Lùi:  IN2 = HIGH (4095), IN1 = PWM đảo (4095 - duty)
  // Dừng: IN1 = 0, IN2 = 0 (Coast) để tránh ngâm dòng cuộn dây khi đứng yên

  // --- BÁNH TRÁI ---
  if (speedDC_left > 0) {
    writeMotorPwm(PIN_MOTOR_LEFT_IN1, LEDC_CH_LEFT_IN1, 4095);
    writeMotorPwm(PIN_MOTOR_LEFT_IN2, LEDC_CH_LEFT_IN2, 4095 - (uint32_t)speedDC_left);
  } else if (speedDC_left < 0) {
    uint32_t duty = (uint32_t)(-speedDC_left);
    writeMotorPwm(PIN_MOTOR_LEFT_IN1, LEDC_CH_LEFT_IN1, 4095 - duty);
    writeMotorPwm(PIN_MOTOR_LEFT_IN2, LEDC_CH_LEFT_IN2, 4095);
  } else {
    writeMotorPwm(PIN_MOTOR_LEFT_IN1, LEDC_CH_LEFT_IN1, 0);
    writeMotorPwm(PIN_MOTOR_LEFT_IN2, LEDC_CH_LEFT_IN2, 0);
  }

  // --- BÁNH PHẢI ---
  if (speedDC_right > 0) {
    writeMotorPwm(PIN_MOTOR_RIGHT_IN1, LEDC_CH_RIGHT_IN1, 4095);
    writeMotorPwm(PIN_MOTOR_RIGHT_IN2, LEDC_CH_RIGHT_IN2, 4095 - (uint32_t)speedDC_right);
  } else if (speedDC_right < 0) {
    uint32_t duty = (uint32_t)(-speedDC_right);
    writeMotorPwm(PIN_MOTOR_RIGHT_IN1, LEDC_CH_RIGHT_IN1, 4095 - duty);
    writeMotorPwm(PIN_MOTOR_RIGHT_IN2, LEDC_CH_RIGHT_IN2, 4095);
  } else {
    writeMotorPwm(PIN_MOTOR_RIGHT_IN1, LEDC_CH_RIGHT_IN1, 0);
    writeMotorPwm(PIN_MOTOR_RIGHT_IN2, LEDC_CH_RIGHT_IN2, 0);
  }
}

void handleAndSpeed(int angle, int speed1) {
  s_cmdSpeed = speed1;
  int speedLeft  = speed1 + angle;
  int speedRight = speed1 - angle;

  // Giữ tỉ lệ phân bổ công suất lái khi một bánh chạm trần 4095
  if (speedLeft > 4095) {
    speedRight -= (speedLeft - 4095);
    speedLeft = 4095;
  } else if (speedRight > 4095) {
    speedLeft -= (speedRight - 4095);
    speedRight = 4095;
  }

  // speed_run_closed sẽ tự clamp âm về -MAX_PLUG_PWM và bù vòng kín vận tốc
  speed_run_closed(speedLeft, speedRight);
}
