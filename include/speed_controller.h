#pragma once
#include <Arduino.h>

struct MotorFFParams {
    float ks;       // Tĩnh ma sát PWM
    float kv;       // Hệ số vận tốc PWM per (mm/s)
    float ka;       // Hệ số gia tốc PWM per (mm/s^2)
    float deadzone; // Vùng chết PWM
};

struct SpeedControllerOutput {
    int pwmL;       // PWM xuất động cơ trái [-255, 255]
    int pwmR;       // PWM xuất động cơ phải [-255, 255]
    float uPI;      // Thành phần PI chung
    float ffL;      // Feedforward bánh trái
    float ffR;      // Feedforward bánh phải
};

void speedControllerInit();
void speedControllerReset();

// Cập nhật tham số Feedforward riêng từng motor
void speedControllerSetMotorParams(const MotorFFParams &left, const MotorFFParams &right);

// Tính toán điều khiển tốc độ: Feedforward từng bánh + PI chung cho vận tốc tịnh tiến
SpeedControllerOutput speedControllerCompute(
    float vTargetLinearMmS, // Tốc độ đích tịnh tiến trung bình [mm/s]
    float vTargetLMmS,      // Tốc độ đích bánh trái [mm/s]
    float vTargetRMmS,      // Tốc độ đích bánh phải [mm/s]
    float aTargetMmS2,      // Gia tốc đích [mm/s^2]
    float vMeasLMmS,        // Tốc độ đo bánh trái [mm/s]
    float vMeasRMmS,        // Tốc độ đo bánh phải [mm/s]
    float dtS,              // Thời gian lấy mẫu [s]
    float vbatMv,           // Điện áp pin thực tế [mV]
    float vnomMv = 11100.0f,// Điện áp danh định [mV] (3S LiPo ~ 11.1V)
    bool freezeIntegral = false
);
