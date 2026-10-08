#pragma once
#include <Arduino.h>

// ==========================================
// 1. Motor & Physical Specifications
// ==========================================
#define MOTOR_MODEL_INFO        "JGB37-520 12V 1590RPM"
#define GEAR_REDUCTION_RATIO    6.25f                       // Gear ratio for 1590RPM variant (6.25:1)
#define MOTOR_HALL_PPR          11.0f                       // Rotor Hall pulse count per revolution
#define ENCODER_TICKS_PER_REV   (MOTOR_HALL_PPR * GEAR_REDUCTION_RATIO * 4.0f) // 275.0 ticks per wheel rev (4x Quadrature)
#define WHEEL_DIAMETER_MM       68.0f                       // Wheel diameter (mm)
#define MAX_MOTOR_RPM           1590.0f                     // Max motor RPM at 12V

// Chassis Geometry (mm)
#define SENSOR_BAR_WIDTH_MM     83.0f                       // 83mm sensor bar span (center Ch0 to center Ch7)
#define SENSOR_PITCH_MM         (SENSOR_BAR_WIDTH_MM / 7.0f)// 11.857 mm pitch between sensor centers
#define SENSOR_EXTRAPOLATE_POS  4.25f                       // Extrapolated index (4.0 - 4.5) when line leaves bar edge
#define SENSOR_EXTRAPOLATE_MM   (SENSOR_EXTRAPOLATE_POS * SENSOR_PITCH_MM) // ~50.39 mm
#define SENSOR_OVERHANG_MM      160.0f                      // 160mm distance from sensor bar to axle
#define TRACK_WIDTH_MM          160.0f                      // 160mm wheelbase (wheel-to-wheel)

// Speed & Control Parameters (Open-Loop PWM Fallback)
#define BASE_RUN_PWM            145                         // Cruising speed on straight (PWM)
#define MIN_CORNER_SPEED        125                         // Instant speed cut in sharp corners (PWM)
#define MAX_STEER_PWM           205                         // Maximum steering authority on line tracking (boosted for sharp curves)
#define DEFAULT_D_FILTER_ALPHA  0.45f                       // Low-pass EMA filter coefficient for derivative

// Closed-Loop Speed Control (mm/s via Encoders)
#define ENABLE_CLOSED_LOOP_SPEED    true                    // Set true for constant mm/s closed-loop, false for open-loop PWM
#define TARGET_BOOST_SPEED_MMS      1400.0f                 // Rocket boost speed on confirmed straights (mm/s)
#define TARGET_BASE_SPEED_MMS       900.0f                  // Target cruising speed on normal straights (mm/s)
#define TARGET_CORNER_SPEED_MMS     550.0f                  // Target speed in curves / flanks (mm/s)
#define SPEED_FF_GAIN               0.115f                  // Feedforward gain: PWM duty per mm/s
#define FF_OFFSET_PWM               0.0f                    // Feedforward deadband offset PWM (dz)
#define SPEED_KP_GAIN               0.16f                   // Proportional gain for speed error
#define SPEED_KI_GAIN               0.75f                   // Integral gain for battery voltage drop compensation
#define SPEED_MAX_I_PWM             40.0f                   // Anti-windup clamp for integral PWM contribution

// Tuned PID for 160mm Overhang Lever Arm
#define KP_GAIN                 20.0f                       // Proportional gain for linear tracking
#define KD_GAIN                 68.0f                       // Angular rate damping to prevent whip across center
#define K_CURVE_GAIN            14.5f                       // Progressive cornering boost to prevent understeer at corner entry
#define CONTROL_LOOP_MS         4                           // 4 ms deterministic loop (250 Hz - rock solid for 8ch MUX + encoders)
#define SENSOR_CENTER_TRIM      0.0f                        // Fine-tune center line bias (e.g. -0.2 to shift left)

// Safety Auto-Stop Timeout
#define LINE_LOST_TIMEOUT_MS    700                         // Stop motors if line lost for >700ms

// Encoder direction invert flags (swap if encoder ticks count backwards)
#define INVERT_LEFT_ENCODER     false
#define INVERT_RIGHT_ENCODER    false

// Motor direction invert flags (swap if motor runs backward)
#define INVERT_LEFT_MOTOR       false
#define INVERT_RIGHT_MOTOR      false

// ==========================================
// 2. Hardware Pinout
// ==========================================
#define PIN_MOTOR_LEFT_IN1      18
#define PIN_MOTOR_LEFT_IN2      19
#define PIN_MOTOR_RIGHT_IN1     26
#define PIN_MOTOR_RIGHT_IN2     25

// Motor Control Mode (DRV8871)
#define MOTOR_SLOW_DECAY        true                        // Active Back-EMF dynamic braking in PWM off-phase

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
#define MUX_SWITCH_DELAY_US     25                          // 25us settling time per channel
#define MUX_ADC_SAMPLES         4                           // 4 samples averaged per channel
#define SENSOR_FLOOR_THRESH     180.0f                      // Continuous floor threshold (180/1000)
#define SENSOR_MIN_DENOM        100.0f                      // Min total denominator sum to detect line

// Pushbuttons (Hardware RC Debounced)
#define PIN_BTN_SW1             13                          // SW1: Calibrate (idle) / E-Stop
#define PIN_BTN_SW2             14                          // SW2: Toggle Start / Stop
#define PIN_BTN_SW3             27                          // SW3: Speed mode / Lap

// I2C OLED 1.3" (128x64)
#define PIN_I2C_SDA             21
#define PIN_I2C_SCL             22
#define OLED_I2C_ADDR           0x3C                        // Standard I2C address
#define OLED_DRIVER_SH1106      1                           // 1 for 1.3" SH1106 (standard 1.3" OLED), 0 for SSD1306
#define OLED_REFRESH_HZ         10                          // Background refresh rate on Core 0 (10 Hz)

// ==========================================
// 3. PWM / LEDC Configuration
// ==========================================
#define PWM_FREQ_HZ             20000                       // 20 kHz ultrasonic carrier
#define PWM_RES_BITS            8                           // 8-bit resolution (0-255)
#define LEDC_CH_LEFT_IN1        0
#define LEDC_CH_LEFT_IN2        1
#define LEDC_CH_RIGHT_IN1       2
#define LEDC_CH_RIGHT_IN2       3

// ==========================================
// 4. Sensor Geometry Weights (83mm Bar)
// Centers at ±5.9mm, ±17.8mm, ±29.6mm, ±41.5mm
// Normalized scale: -3.5 (M1) to +3.5 (M8)
// ==========================================
const float SENSOR_WEIGHTS[8] = {
    -3.5f, // Ch 0: M1 (Leftmost,  -41.5mm)
    -2.5f, // Ch 1: M2 (-29.6mm)
    -1.5f, // Ch 2: M3 (-17.8mm)
    -0.5f, // Ch 3: M4 (-5.9mm,  Center-Left)
     0.5f, // Ch 4: M5 (+5.9mm,  Center-Right)
     1.5f, // Ch 5: M6 (+17.8mm)
     2.5f, // Ch 6: M7 (+29.6mm)
     3.5f  // Ch 7: M8 (Rightmost, +41.5mm)
};

// ==========================================
// 5. Dynamic Car Configuration (Flash NVS Stored)
// ==========================================
struct CarConfig {
    float boostSpeedMmS;   // 1200 - 2200 (step 50) - Rocket boost on straight track
    float baseSpeedMmS;    // 800 - 1800 (step 50)  - Normal cruising speed
    float cornerSpeedMmS;  // 600 - 1200 (step 50)  - Safe corner entry speed
    float kpGain;          // 10.0 - 50.0 (step 1.0)
    float kdGain;          // 10.0 - 70.0 (step 1.0)
    float kCurveGain;      // 1.0 - 15.0 (step 0.5)
    bool  activeBrakeEn;   // true / false (Electronic ABS)
};

void configLoad(CarConfig &cfg);
void configSave(const CarConfig &cfg);
void configResetDefaults(CarConfig &cfg);

