#pragma once
#include <Arduino.h>

// ==========================================
// 1. Physical Specifications
// ==========================================
#define GEAR_REDUCTION_RATIO    6.25f
#define MOTOR_HALL_PPR          11.0f
#define ENCODER_TICKS_PER_REV   (MOTOR_HALL_PPR * GEAR_REDUCTION_RATIO * 4.0f)
#define WHEEL_DIAMETER_MM       68.0f
#define MM_PER_TICK             ((3.14159265f * WHEEL_DIAMETER_MM) / ENCODER_TICKS_PER_REV)
#define TRACK_WIDTH_MM          160.0f

// Sensor Bar Geometry (mm)
#define SENSOR_BAR_WIDTH_MM     83.0f
#define SENSOR_PITCH_MM         (SENSOR_BAR_WIDTH_MM / 7.0f)
#define SENSOR_EXTRAPOLATE_POS  4.25f
#define SENSOR_EXTRAPOLATE_MM   (SENSOR_EXTRAPOLATE_POS * SENSOR_PITCH_MM)
#define SENSOR_CENTER_TRIM      0.0f

// Deterministic Loop & Safety
#define CONTROL_LOOP_MS         4
#define LINE_LOST_TIMEOUT_MS    700

// Pure PD Defaults
#define DEFAULT_BOOST_SPEED     160.0f
#define DEFAULT_BASE_SPEED      130.0f
#define DEFAULT_CORNER_SPEED    90.0f
#define KP_GAIN                 1.0f
#define KD_GAIN                 15.0f
#define K_CURVE_GAIN            0.0f

// Direction Invert Flags (Đổi thành true nếu bánh xe quay ngược chiều)
#define INVERT_LEFT_ENCODER     false
#define INVERT_RIGHT_ENCODER    false
#define INVERT_LEFT_MOTOR       false
#define INVERT_RIGHT_MOTOR      false

// ==========================================
// 2. Hardware Pinout
// ==========================================
#define PIN_MOTOR_LEFT_IN1      25
#define PIN_MOTOR_LEFT_IN2      26
#define PIN_MOTOR_RIGHT_IN1     19
#define PIN_MOTOR_RIGHT_IN2     18

// Encoders (JGB37-520)
#define PIN_ENC_LEFT_A          32
#define PIN_ENC_LEFT_B          33
#define PIN_ENC_RIGHT_A         34
#define PIN_ENC_RIGHT_B         35

// 8-Channel Analog Multiplexer (CD74HC4051)
#define PIN_MUX_SIG             36
#define PIN_MUX_S0              16
#define PIN_MUX_S1              17
#define PIN_MUX_S2              23
#define MUX_SWITCH_DELAY_US     80
#define MUX_ADC_SAMPLES         2

// Pushbuttons
#define PIN_BTN_SW1             13
#define PIN_BTN_SW2             14
#define PIN_BTN_SW3             27

// I2C OLED 1.3" (128x64)
#define PIN_I2C_SDA             21
#define PIN_I2C_SCL             22
#define OLED_I2C_ADDR           0x3C
#define OLED_DRIVER_SH1106      1
#define OLED_REFRESH_HZ         10

// ==========================================
// 3. PWM Configuration
// ==========================================
#define PWM_FREQ_HZ             19000
#define PWM_RES_BITS            12
#define LEDC_CH_LEFT_IN1        0
#define LEDC_CH_LEFT_IN2        1
#define LEDC_CH_RIGHT_IN1       2
#define LEDC_CH_RIGHT_IN2       3

// ==========================================
// 4. Sensor Geometry Weights
// ==========================================
const float SENSOR_WEIGHTS[8] = {
    -3.5f, -2.5f, -1.5f, -0.5f,
     0.5f,  1.5f,  2.5f,  3.5f
};

// ==========================================
// 5. Dynamic Car Configuration (Flash NVS)
// ==========================================
struct CarConfig {
    float boostSpeed;
    float baseSpeed;
    float cornerSpeed;
    float kpGain;
    float kdGain;
    float kCurveGain;
    bool  activeBrakeEn;
};

void configLoad(CarConfig &cfg);
void configSave(const CarConfig &cfg);
void configResetDefaults(CarConfig &cfg);

// Menu Parameter Bounds
#define CFG_BOOST_SPD_MIN       50.0f
#define CFG_BOOST_SPD_MAX       255.0f
#define CFG_BASE_SPD_MIN        40.0f
#define CFG_BASE_SPD_MAX        255.0f
#define CFG_CORNER_SPD_MIN      30.0f
#define CFG_CORNER_SPD_MAX      255.0f
#define CFG_KP_MIN              0.0f
#define CFG_KP_MAX              30.0f
#define CFG_KD_MIN              0.0f
#define CFG_KD_MAX              100.0f
#define CFG_KC_MIN              0.0f
#define CFG_KC_MAX              20.0f