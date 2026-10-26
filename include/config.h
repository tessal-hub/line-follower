#pragma once
#include <Arduino.h>

// ==========================================
// 1. Physical Specifications
// ==========================================
#define GEAR_REDUCTION_RATIO    6.25f
#define MOTOR_HALL_PPR          11.0f
#define ENCODER_TICKS_PER_REV   (MOTOR_HALL_PPR * GEAR_REDUCTION_RATIO * 4.0f) // ~275 ticks/rev
#define WHEEL_DIAMETER_MM       68.0f
#define MM_PER_TICK             ((3.14159265f * WHEEL_DIAMETER_MM) / ENCODER_TICKS_PER_REV) // ~0.7768 mm/tick

// Sensor Bar Geometry (mm)
#define SENSOR_BAR_WIDTH_MM     83.0f
#define SENSOR_PITCH_MM         (SENSOR_BAR_WIDTH_MM / 7.0f)

// Direction Invert Flags (Đổi thành true nếu đảo chiều phần cứng)
#define INVERT_LEFT_ENCODER     false
#define INVERT_RIGHT_ENCODER    false
#define INVERT_LEFT_MOTOR       false
#define INVERT_RIGHT_MOTOR      false

// ==========================================
// 2. Hardware Pinout
// ==========================================
// Motor Driver (DRV8871 H-Bridge)
#define PIN_MOTOR_LEFT_IN1      25
#define PIN_MOTOR_LEFT_IN2      26
#define PIN_MOTOR_RIGHT_IN1     19
#define PIN_MOTOR_RIGHT_IN2     18

// Encoders (JGB37-520)
// LƯU Ý PHẦN CỨNG: GPIO 34 và 35 là chân INPUT ONLY của ESP32, KHÔNG có điện trở pull-up nội!
// Bắt buộc phải hàn trở kéo lên ngoài (4.7k - 10k lên 3.3V) cho PIN_ENC_RIGHT_A & B.
#define PIN_ENC_LEFT_A          32
#define PIN_ENC_LEFT_B          33
#define PIN_ENC_RIGHT_A         34
#define PIN_ENC_RIGHT_B         35

// 8-Channel Analog Multiplexer (CD74HC4051)
#define PIN_MUX_SIG             36
#define PIN_MUX_S0              16
#define PIN_MUX_S1              17
#define PIN_MUX_S2              23
#define MUX_SWITCH_DELAY_US     2   // HC4051 t_on ~100ns; 2µs dư an toàn cho tụ S&H ADC
#define MUX_ADC_SAMPLES         1   // 1 mẫu đọc nhanh mỗi kênh

// Pushbuttons (active LOW with internal pull-up)
#define PIN_BTN_SW1             13  // Calib / Dừng khẩn cấp / Cancel
#define PIN_BTN_SW2             14  // Chạy Base Speed / Giảm giá trị
#define PIN_BTN_SW3             27  // Chạy Fast Speed / Giữ để vào Menu / Tăng giá trị

// Status Output (Buzzer & RGB LED)
#define PIN_BUZZER              4
#define PIN_LED_R               5
#define PIN_LED_G               2
#define PIN_LED_B               15

// I2C OLED 1.3" (128x64)
#define PIN_I2C_SDA             21
#define PIN_I2C_SCL             22
#define OLED_I2C_ADDR           0x3C
#define OLED_DRIVER_SH1106      1

// ==========================================
// 3. PWM Configuration (12-bit @ 16kHz)
// ESP32 APB_CLK = 80MHz. Với 12-bit (4096 mức), tần số tối đa: 80MHz / 4096 = 19.531 kHz.
// 16000 Hz nằm hoàn toàn trong giới hạn phần cứng, triệt tiêu tiếng rít động cơ (trên ngưỡng tai người).
// ==========================================
#define PWM_FREQ_HZ             16000
#define PWM_RES_BITS            12
#define LEDC_CH_LEFT_IN1        0
#define LEDC_CH_LEFT_IN2        1
#define LEDC_CH_RIGHT_IN1       2
#define LEDC_CH_RIGHT_IN2       3

// Giới hạn xung phanh ngược (Plugging Duty Cap) chống sụt áp 12V brownout ESP32
#define MAX_PLUG_PWM            3000

// Hệ số quy đổi tốc độ dài thực (mm/s) sang PWM tương đương
#define PWM_PER_MMS             1.45f

// ==========================================
// 3b. Master Control Loop & Telemetry
// 2kHz: giảm một nửa độ trễ pipeline sensor->PWM (tối đa 500µs thay vì 1ms).
// D-term vẫn cập nhật mỗi 2ms để giữ nguyên thang tuning Kd cũ.
// ==========================================
#define CONTROL_LOOP_HZ         2000
#define CONTROL_PERIOD_US       (1000000UL / CONTROL_LOOP_HZ)

// Telemetry: 115200 baud chặn vòng lặp ~9ms/100 ký tự (mù 17mm tại 2m/s!).
// 921600 baud + chu kỳ 200ms: dòng ~70 ký tự vừa FIFO TX UART, không block.
#define TELEMETRY_BAUD          921600
#define TELEMETRY_PERIOD_MS     200

// ==========================================
// 4. Finish Line / Crossbar Detection
// LƯU Ý: các ngưỡng tính bằng SỐ LẦN GỌI read_sensor() chạy ở 2kHz,
// nên giá trị = thời gian(ms) x 2 để giữ đúng thời gian thực.
// ==========================================
#define FINISH_LINE_ENABLE          true
#define FINISH_LINE_MIN_TICKS       24      // 12ms thực (12 chu kỳ x 2kHz) thấy cả 7..8 mắt đen
#define FINISH_LINE_IGNORE_START_MS 2000    // Bỏ qua vạch xuất phát trong 2 giây đầu sau khi bấm chạy
#define CENTER_CLEAR_TICKS          24      // 12ms thực: vạch về giữa liên tục -> xóa RememberLine
