#pragma once
#include <Arduino.h>

#define MAX_TRACK_POINTS 500 // Tối đa 500 điểm map * 0.05m = 25 mét đường đua
#define TRACK_DS_M       0.05f // Bước quãng đường delta_s = 50 mm (5 cm)

struct TrackPoint {
    float kappa;  // Độ cong đo được [m^-1]
    float vMax;   // Giới hạn gia tốc ngang v_max = sqrt(a_lat / |kappa|) [mm/s]
    float vPlan;  // Tốc độ quy hoạch sau forward/backward pass [mm/s]
};

struct PlannerConfig {
    float aLatMax;       // Gia tốc ngang cực đại [m/s^2] (ví dụ 3.5 m/s^2)
    float aAccelMax;     // Gia tốc dọc cực đại a0 [m/s^2] (ví dụ 2.5 m/s^2)
    float aBrakeMax;     // Gia tốc phanh cực đại [m/s^2] (ví dụ 4.0 m/s^2)
    float vMaxMotor;     // Tốc độ tối đa động cơ [mm/s] (1800 mm/s)
    float vMinCorner;    // Sàn tốc độ tối thiểu cho cua hẹp [mm/s] (400 mm/s)
    float tLagS;         // Thời gian trễ bù phanh sớm t_lag [s] (0.040s = 40ms)
};

void trackProfileInit();
void trackProfileReset();

// Bắt đầu / kết thúc ghi map đường đua
void trackProfileStartRecording();
void trackProfileStopRecording();
bool trackProfileIsRecorded();

// Cập nhật điểm map trong lúc chạy lap thăm dò
void trackProfileRecordStep(float distS, float measuredKappa, float confidence);

// Chạy 2 lượt Forward Pass & Backward Pass để tạo Speed Profile hoàn chỉnh
void trackProfileGenerateProfile(const PlannerConfig &cfg);

// Lấy tốc độ quy hoạch tại quãng đường s [m]
float trackProfileGetTargetSpeed(float sM);

// Fallback phản ứng khi chưa có map hoặc đang thăm dò
float trackProfileComputeReactiveFallback(
    float errorMm,
    float dErrorMmS,
    float confidence,
    float baseSpeedMmS,
    float cornerSpeedMmS,
    float currentSpeedMmS
);

// Safety Override: Kiểm tra an toàn khẩn cấp
bool trackProfileCheckSafetyOverride(
    float errorMm,
    float confidence,
    float eLimitMm = 35.0f,
    float confLimit = 0.30f
);
