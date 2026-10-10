#pragma once
#include <Arduino.h>

// ==========================================
// 1. Physical Specifications
// ==========================================
#define GEAR_REDUCTION_RATIO    6.25f                       // Gear ratio for 1590RPM variant (6.25:1)
#define MOTOR_HALL_PPR          11.0f                       // Rotor Hall pulse count per revolution
#define ENCODER_TICKS_PER_REV   (MOTOR_HALL_PPR * GEAR_REDUCTION_RATIO * 4.0f) // 275.0 ticks per wheel rev
#define WHEEL_DIAMETER_MM       68.0f                       // Wheel diameter (mm)
#define MM_PER_TICK             ((3.14159265f * WHEEL_DIAMETER_MM) / ENCODER_TICKS_PER_REV) // ~0.7768 mm/tick
#define TRACK_WIDTH_MM          160.0f                      // Wheel-to-wheel distance (mm)

// Sensor Bar Geometry (mm)
#define SENSOR_BAR_WIDTH_MM     83.0f                       // 83mm sensor bar span (center Ch0 to Ch7)
#define SENSOR_PITCH_MM         (SENSOR_BAR_WIDTH_MM / 7.0f)// 11.857 mm pitch between sensors
#define SENSOR_EXTRAPOLATE_POS  4.25f                       // Extrapolated index when line leaves bar edge
#define SENSOR_EXTRAPOLATE_MM   (SENSOR_EXTRAPOLATE_POS * SENSOR_PITCH_MM) // ~50.39 mm
#define SENSOR_CENTER_TRIM      0.0f                        // Center trim offset (mm)

// Deterministic Loop & Safety
#define CONTROL_LOOP_MS         4                           // 4 ms loop (250 Hz)
#define LINE_LOST_TIMEOUT_MS    700                         // Stop motors if line lost for >700ms

// Pure PD Defaults (Direct PWM [0..255])
#define DEFAULT_BOOST_SPEED     160.0f                      // Boost PWM on straight line
#define DEFAULT_BASE_SPEED      130.0f                      // Base cruising PWM
#define DEFAULT_CORNER_SPEED    90.0f                       // Cornering entry PWM
#define KP_GAIN                 4.5f                        // Direct P multiplier: P = Kp * error_mm
#define KD_GAIN                 22.0f                       // Direct D multiplier: D = Kd * delta_error_mm
#define K_CURVE_GAIN            0.0f                        // Optional extra gain on sharp turns

// Direction Invert Flags
#define INVERT_LEFT_ENCODER     false
#define INVERT_RIGHT_ENCODER    false
#define INVERT_LEFT_MOTOR       false
#define INVERT_RIGHT_MOTOR      false

// ==========================================
// 2. Hardware Pinout
// ==========================================
#define PIN_MOTOR_LEFT_IN1      18
#define PIN_MOTOR_LEFT_IN2      19
#define PIN_MOTOR_RIGHT_IN1     26
#define PIN_MOTOR_RIGHT_IN2     25

// Encoders (JGB37-520)
#define PIN_ENC_LEFT_A          32
#define PIN_ENC_LEFT_B          33
#define PIN_ENC_RIGHT_A         34
#define PIN_ENC_RIGHT_B         35

// 8-Channel Analog Multiplexer (CD74HC4051)
#define PIN_MUX_SIG             36                          // ADC1_CH0 (SENSOR_VP)
#define PIN_MUX_S0              16
#define PIN_MUX_S1              17
#define PIN_MUX_S2              23
#define MUX_SWITCH_DELAY_US     80                          // Settling time per channel (us, eliminates RC crosstalk)
#define MUX_ADC_SAMPLES         2                           // Fast 2x oversampling

// Pushbuttons (Hardware RC Debounced)
#define PIN_BTN_SW1             13                          // SW1: Calibrate (idle) / E-Stop
#define PIN_BTN_SW2             14                          // SW2: Toggle Start / Stop
#define PIN_BTN_SW3             27                          // SW3: OLED Menu

// I2C OLED 1.3" (128x64)
#define PIN_I2C_SDA             21
#define PIN_I2C_SCL             22
#define OLED_I2C_ADDR           0x3C
#define OLED_DRIVER_SH1106      1                           // 1 for SH1106, 0 for SSD1306
#define OLED_REFRESH_HZ         10                          // Background refresh on Core 0

// ==========================================
// 3. PWM Configuration
// ==========================================
#define PWM_FREQ_HZ             20000                       // 20 kHz carrier
#define PWM_RES_BITS            8                           // 8-bit resolution (0-255)
#define LEDC_CH_LEFT_IN1        0
#define LEDC_CH_LEFT_IN2        1
#define LEDC_CH_RIGHT_IN1       2
#define LEDC_CH_RIGHT_IN2       3

// ==========================================
// 4. Sensor Geometry Weights
// Normalized: -3.5 (M1) to +3.5 (M8)
// ==========================================
const float SENSOR_WEIGHTS[8] = {
    -3.5f, -2.5f, -1.5f, -0.5f,
     0.5f,  1.5f,  2.5f,  3.5f
};

// ==========================================
// 5. Dynamic Car Configuration (Flash NVS)
// ==========================================
struct CarConfig {
    float boostSpeed;      // PWM: 50 - 255
    float baseSpeed;       // PWM: 40 - 255
    float cornerSpeed;     // PWM: 30 - 255
    float kpGain;          // 0.0 - 30.0 (Direct multiplier on error_mm)
    float kdGain;          // 0.0 - 100.0 (Direct multiplier on delta_error_mm)
    float kCurveGain;      // 0.0 - 20.0 (Optional curve boost)
    bool  activeBrakeEn;   // Dynamic braking (true / false)
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
