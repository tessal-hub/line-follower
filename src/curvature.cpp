#include "curvature.h"
#include "config.h"

static float lastErrorM = 0.0f;
static float filteredDeDtM = 0.0f;
static bool isFirstCycle = true;

// Floor speed tối thiểu [m/s] để không mất quyền lái khi xe đi chậm
static const float V_FLOOR_MS = 0.25f; // 250 mm/s

void curvatureInit() {
    curvatureReset();
}

void curvatureReset() {
    lastErrorM = 0.0f;
    filteredDeDtM = 0.0f;
    isFirstCycle = true;
}

CurvatureOutput curvatureCompute(
    float errorM,
    float dtS,
    float targetSpeedMmS,
    float kpCurvature,
    float kdCurvature,
    float kCurveNonlinear,
    float maxWheelSpeedMmS,
    bool sharpTurnMode
) {
    CurvatureOutput out;

    if (dtS <= 0.0005f || dtS > 0.05f) {
        dtS = 0.004f; // Fallback 250 Hz (4ms)
    }

    // 1. Tính đạo hàm sai số ngang de/dt [m/s] với bộ lọc thông thấp nhẹ
    float dRaw = 0.0f;
    if (!isFirstCycle) {
        dRaw = (errorM - lastErrorM) / dtS;
    } else {
        isFirstCycle = false;
        dRaw = 0.0f;
    }
    lastErrorM = errorM;

    // Lọc EMA nhẹ (alpha = 0.40) để triệt tiêu nhiễu lượng tử hóa mà không tăng trễ pha
    filteredDeDtM += 0.40f * (dRaw - filteredDeDtM);
    out.dErrorMs = filteredDeDtM;

    // 2. Tính độ cong yêu cầu kappa_cmd [m^-1]
    // Thành phần phi tuyến kCurve: khi lệch ngang |e| tăng (vào khúc cua),
    // kCurve tự động tăng quyền lái tỉ lệ với độ lệch (|e| / 20mm) để chống thiếu lái triệt để
    float eRatio = fabsf(errorM) / 0.020f; // 20mm là mốc chuẩn vào cua
    float effectiveKp = kpCurvature + kCurveNonlinear * eRatio;
    float kappa = effectiveKp * errorM + kdCurvature * filteredDeDtM;

    // Trong chế độ SHARP_TURN: tăng quyền lái thêm 60% để ép đầu xe gập cua dứt khoát
    if (sharpTurnMode) {
        kappa *= 1.60f;
    }

    // Nới rộng giới hạn bán kính quay: cho phép kappa lên tới 22.0 m^-1 (R_min ~ 4.5cm)
    // Giúp bánh trong có thể phanh đứng hoặc lùi nhẹ để xe xoay ngoắt góc 90° cực nhanh
    kappa = constrain(kappa, -22.0f, 22.0f);
    out.kappaCmd = kappa;

    // 3. Quy đổi sang vận tốc góc omega_cmd [rad/s] = v * kappa
    // Dùng v_floor để bảo toàn quyền lái omega khi v nhỏ
    float vLinearMs = (targetSpeedMmS * 0.001f);
    float vSteeringEffectiveMs = (vLinearMs > V_FLOOR_MS) ? vLinearMs : V_FLOOR_MS;
    float omega = vSteeringEffectiveMs * kappa; // [rad/s]
    out.omegaCmd = omega;

    // 4. Phân bổ tốc độ vi sai bánh xe (Differential Kinematics)
    // Track width B = 0.16 m (160 mm)
    // v_left  = v - omega * B / 2  [m/s]
    // v_right = v + omega * B / 2  [m/s]
    float trackWidthM = (TRACK_WIDTH_MM * 0.001f);
    float diffSpeedMs = (omega * trackWidthM * 0.5f);

    float vLeftMs  = vLinearMs - diffSpeedMs;
    float vRightMs = vLinearMs + diffSpeedMs;

    float vLeftMmS  = vLeftMs * 1000.0f;
    float vRightMmS = vRightMs * 1000.0f;

    // 5. Giới hạn tốc độ bánh xe: nếu một bánh vượt v_wheel_max thì scale cả hai bánh theo cùng tỉ lệ
    // để BẢO TOÀN ĐỘ CONG KAPPA (không bị xén cắt méo mó góc lái!)
    float maxReq = fmaxf(fabsf(vLeftMmS), fabsf(vRightMmS));
    if (maxReq > maxWheelSpeedMmS && maxReq > 0.001f) {
        float scale = maxWheelSpeedMmS / maxReq;
        vLeftMmS  *= scale;
        vRightMmS *= scale;
        targetSpeedMmS *= scale;
    }

    out.targetSpeedMmS = targetSpeedMmS;
    out.vLeftMmS  = vLeftMmS;
    out.vRightMmS = vRightMmS;

    return out;
}
