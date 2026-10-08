#pragma once
#include <Arduino.h>

struct CurvatureOutput {
    float kappaCmd;       // Yêu cầu độ cong kappa [m^-1]
    float omegaCmd;       // Vận tốc góc [rad/s]
    float targetSpeedMmS; // Tốc độ trung bình sau khi scale [mm/s]
    float vLeftMmS;       // Tốc độ đích bánh trái [mm/s]
    float vRightMmS;      // Tốc độ đích bánh phải [mm/s]
    float dErrorMs;       // de/dt đã lọc [m/s]
};

void curvatureInit();
void curvatureReset();

// Tính toán độ cong theo Pure Pursuit và phân bổ vận tốc vi sai
CurvatureOutput curvatureCompute(
    float errorM,           // Lệch ngang e [m]
    float dtS,              // Thời gian lấy mẫu [s]
    float targetSpeedMmS,   // Tốc độ đích tuyến tính [mm/s]
    float kpCurvature,      // Kp [m^-2] (Lý thuyết: 2/L^2 ~ 78 m^-2)
    float kdCurvature,      // Kd [s/m^2]
    float kCurveNonlinear,  // Thành phần phi tuyến kCurve chống thiếu lái khi vào cua
    float maxWheelSpeedMmS, // Tốc độ tối đa của bánh xe [mm/s]
    bool sharpTurnMode = false // Chế độ cua gắt
);
