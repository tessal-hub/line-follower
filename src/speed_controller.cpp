#include "speed_controller.h"
#include "config.h"

// Thông số Feedforward cho động cơ JGB37-520 12V 1590RPM
// Đã gộp deadzone vào ks (duy nhất 1 thành phần ma sát tĩnh ~12 PWM, mượt mà trong dải ±25 mm/s)
// ka = 0.002f kết hợp cùng Slew Limiter giúp tăng tốc dứt khoát mà không sinh nhiễu bang-bang
static MotorFFParams ffLeft = {
    .ks = 15.0f,
    .kv = 0.115f,
    .ka = 0.002f,
    .deadzone = 0.0f
};

static MotorFFParams ffRight = {
    .ks = 15.0f,
    .kv = 0.115f,
    .ka = 0.002f,
    .deadzone = 0.0f
};

// PI chung cho vận tốc tịnh tiến
static float speedIntegral = 0.0f;
static const float SPEED_KP = 0.16f; // PWM per (mm/s error)
static const float SPEED_KI = 0.75f; // PWM per (mm/s * s)
static const float MAX_INTEGRAL_PWM = 40.0f;

void speedControllerInit() {
    speedControllerReset();
}

void speedControllerReset() {
    speedIntegral = 0.0f;
}

void speedControllerSetMotorParams(const MotorFFParams &left, const MotorFFParams &right) {
    ffLeft = left;
    ffRight = right;
}

static float computeFeedforward(float vTarget, float aTarget, const MotorFFParams &p, float vbatScale) {
    if (fabsf(vTarget) < 1.0f && fabsf(aTarget) < 1.0f) {
        return 0.0f;
    }

    // Khử triệt để hiện tượng giật bước ±20 PWM quanh v=0:
    // Tuyến tính hóa mượt mà ma sát tĩnh trong dải [-25, +25] mm/s
    float friction = 0.0f;
    if (fabsf(vTarget) >= 25.0f) {
        friction = p.ks * ((vTarget > 0.0f) ? 1.0f : -1.0f);
    } else {
        friction = p.ks * (vTarget / 25.0f);
    }

    float ff = (friction + p.kv * vTarget + p.ka * aTarget) * vbatScale;
    return ff;
}

SpeedControllerOutput speedControllerCompute(
    float vTargetLinearMmS,
    float vTargetLMmS,
    float vTargetRMmS,
    float aTargetMmS2,
    float vMeasLMmS,
    float vMeasRMmS,
    float dtS,
    float vbatMv,
    float vnomMv,
    bool freezeIntegral
) {
    SpeedControllerOutput out;

    if (dtS <= 0.0005f || dtS > 0.05f) dtS = 0.004f;

    // 1. Hệ số bù điện áp pin (Battery Voltage Scaling)
    float vbatSafe = fmaxf(vbatMv, 8000.0f); // Bảo vệ chống chia cho 0
    float vbatScale = vnomMv / vbatSafe;

    // 2. Feedforward độc lập từng bánh theo vận tốc & gia tốc đích
    out.ffL = computeFeedforward(vTargetLMmS, aTargetMmS2, ffLeft, vbatScale);
    out.ffR = computeFeedforward(vTargetRMmS, aTargetMmS2, ffRight, vbatScale);

    // 3. Vòng PI điều khiển tốc độ trung bình chung u_PI
    float vMeasAvg = (vMeasLMmS + vMeasRMmS) * 0.5f;
    float errV = vTargetLinearMmS - vMeasAvg;

    if (!freezeIntegral) {
        speedIntegral += SPEED_KI * errV * dtS;
        speedIntegral = constrain(speedIntegral, -MAX_INTEGRAL_PWM, MAX_INTEGRAL_PWM);
    } else {
        // Xả nhẹ tích phân khi đang vào góc gắt
        speedIntegral *= 0.96f;
    }

    // Reset tích phân nếu đổi chiều hoặc dừng hẳn
    if (vTargetLinearMmS <= 20.0f) {
        speedIntegral = 0.0f;
    }

    out.uPI = SPEED_KP * errV + speedIntegral;

    // 4. Kết hợp Kiến trúc đã chốt: PWM = PWM_ff + u_PI (cộng chung cho cả hai bánh)
    float rawPwmL = out.ffL + out.uPI;
    float rawPwmR = out.ffR + out.uPI;

    // 5. Bảo toàn chênh lệch vận tốc góc khi bị bão hòa trần/sàn PWM
    int pwmL = (int)roundf(rawPwmL);
    int pwmR = (int)roundf(rawPwmR);

    int over = max(pwmL, pwmR) - 255;
    if (over > 0) {
        pwmL -= over;
        pwmR -= over;
    }

    int under = -255 - min(pwmL, pwmR);
    if (under > 0) {
        pwmL += under;
        pwmR += under;
    }

    out.pwmL = constrain(pwmL, -255, 255);
    out.pwmR = constrain(pwmR, -255, 255);

    return out;
}
