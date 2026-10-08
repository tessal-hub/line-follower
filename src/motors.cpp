#include "motors.h"
#include "config.h"

static void writePwm(uint8_t pin, uint8_t ch, uint8_t duty) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWrite(pin, duty);
#else
    ledcWrite(ch, duty);
#endif
}

void motorsInit() {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcAttach(PIN_MOTOR_LEFT_IN1, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttach(PIN_MOTOR_LEFT_IN2, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttach(PIN_MOTOR_RIGHT_IN1, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttach(PIN_MOTOR_RIGHT_IN2, PWM_FREQ_HZ, PWM_RES_BITS);
#else
    ledcSetup(LEDC_CH_LEFT_IN1, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(PIN_MOTOR_LEFT_IN1, LEDC_CH_LEFT_IN1);
    ledcSetup(LEDC_CH_LEFT_IN2, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(PIN_MOTOR_LEFT_IN2, LEDC_CH_LEFT_IN2);
    ledcSetup(LEDC_CH_RIGHT_IN1, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(PIN_MOTOR_RIGHT_IN1, LEDC_CH_RIGHT_IN1);
    ledcSetup(LEDC_CH_RIGHT_IN2, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(PIN_MOTOR_RIGHT_IN2, LEDC_CH_RIGHT_IN2);
#endif
    motorsStop();
}

static void applyMotor(uint8_t in1Pin, uint8_t in1Ch, uint8_t in2Pin, uint8_t in2Ch, int pwm, bool invert) {
    if (invert) pwm = -pwm;
    pwm = constrain(pwm, -255, 255);

#if MOTOR_SLOW_DECAY
    // Slow Decay (Drive / Brake) on DRV8871:
    // Low-side FETs short motor coils during PWM off-phase to produce strong electromagnetic Back-EMF braking
    if (pwm > 0) {
        writePwm(in1Pin, in1Ch, 255);
        writePwm(in2Pin, in2Ch, 255 - pwm);
    } else if (pwm < 0) {
        writePwm(in1Pin, in1Ch, 255 + pwm); // pwm is negative: 255 - |pwm|
        writePwm(in2Pin, in2Ch, 255);
    } else {
        // Active dynamic brake when running and receiving 0
        writePwm(in1Pin, in1Ch, 255);
        writePwm(in2Pin, in2Ch, 255);
    }
#else
    if (pwm > 0) {
        writePwm(in1Pin, in1Ch, (uint8_t)pwm);
        writePwm(in2Pin, in2Ch, 0);
    } else if (pwm < 0) {
        writePwm(in1Pin, in1Ch, 0);
        writePwm(in2Pin, in2Ch, (uint8_t)(-pwm));
    } else {
        writePwm(in1Pin, in1Ch, 0);
        writePwm(in2Pin, in2Ch, 0);
    }
#endif
}

void motorsSet(int leftPwm, int rightPwm) {
    applyMotor(PIN_MOTOR_LEFT_IN1, LEDC_CH_LEFT_IN1, PIN_MOTOR_LEFT_IN2, LEDC_CH_LEFT_IN2, leftPwm, INVERT_LEFT_MOTOR);
    applyMotor(PIN_MOTOR_RIGHT_IN1, LEDC_CH_RIGHT_IN1, PIN_MOTOR_RIGHT_IN2, LEDC_CH_RIGHT_IN2, rightPwm, INVERT_RIGHT_MOTOR);
}

void motorsStop() {
    // Coast / Float (Low power, no coil buzzing at standstill)
    writePwm(PIN_MOTOR_LEFT_IN1, LEDC_CH_LEFT_IN1, 0);
    writePwm(PIN_MOTOR_LEFT_IN2, LEDC_CH_LEFT_IN2, 0);
    writePwm(PIN_MOTOR_RIGHT_IN1, LEDC_CH_RIGHT_IN1, 0);
    writePwm(PIN_MOTOR_RIGHT_IN2, LEDC_CH_RIGHT_IN2, 0);
}

void motorsBrake() {
    // Active low-side dynamic brake
    writePwm(PIN_MOTOR_LEFT_IN1, LEDC_CH_LEFT_IN1, 255);
    writePwm(PIN_MOTOR_LEFT_IN2, LEDC_CH_LEFT_IN2, 255);
    writePwm(PIN_MOTOR_RIGHT_IN1, LEDC_CH_RIGHT_IN1, 255);
    writePwm(PIN_MOTOR_RIGHT_IN2, LEDC_CH_RIGHT_IN2, 255);
}

void handleAndSpeed(int steer, int baseSpeed, float absErr, int &outLeft, int &outRight) {
    steer = constrain(steer, -MAX_STEER_PWM, MAX_STEER_PWM);

    // Cho phép bánh trong lùi chủ động sớm hơn: bắt đầu khi |e| > 0.15, lùi tối đa (-160) khi |e| >= 0.70
    float k = constrain((absErr - 0.15f) / 0.55f, 0.0f, 1.0f);
    int minInner = -(int)(160.0f * k);

    int left  = baseSpeed + steer;
    int right = baseSpeed - steer;

    int over = max(left, right) - 255;          // bảo toàn chênh lệch
    if (over > 0) {
        left  -= over;
        right -= over;
    }

    left  = constrain(left,  minInner, 255);
    right = constrain(right, minInner, 255);

    outLeft = left;
    outRight = right;
    motorsSet(left, right);
}

