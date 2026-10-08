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
    // kCurve tự động tăng quyền lái tỉ lệ với độ lệch (|e| / 20mm) để chống thiếu lái
    float absEM = fabsf(errorM);
    float eRatio = absEM / 0.020f; // 20mm là mốc chuẩn vào cua
    float effectiveKp = kpCurvature + kCurveNonlinear * eRatio;

    if (sharpTurnMode) {
        // Cua gắt / góc 90°: Tăng vọt quyền lái tỉ lệ với độ lệch biên
        // Khi e > 20mm, tăng quyền lái từ 1.8x lên đến 3.8x
        float sharpMultiplier = 1.8f + 2.0f * constrain((absEM - 0.020f) / 0.025f, 0.0f, 1.0f);
        effectiveKp *= sharpMultiplier;
    }

    float kappa = effectiveKp * errorM + kdCurvature * filteredDeDtM;

    // Lực bẻ góc dứt khoát khi vạch tràn sát mép hoặc ngoại suy (absEM >= 24mm):
    // Bơm thêm để đảm bảo kappa vượt ngưỡng 12.5 m^-1 (ngưỡng vật lý để bánh trong
    // đảo chiều quay lùi v_inner < 0, giúp xe quay ngoắt góc 90° quanh trục bánh!)
    if (sharpTurnMode && absEM >= 0.024f) {
        float dir = (errorM > 0.0f) ? 1.0f : -1.0f;
        kappa += dir * 8.0f * constrain((absEM - 0.024f) / 0.016f, 0.0f, 1.0f);
    }

    // Giới hạn vật lý độ cong kappa ∈ [-22.0, 22.0] m^-1 (R_min ~ 4.5cm)
    kappa = constrain(kappa, -22.0f, 22.0f);
    out.kappaCmd = kappa;

    // 3. Quy đổi sang vận tốc góc omega_cmd [rad/s] = v * kappa
    // Dùng v_floor để bảo toàn quyền lái omega khi v nhỏ
    float vLinearMs = (targetSpeedMmS * 0.001f);
    float vSteeringEffectiveMs = (vLinearMs > V_FLOOR_MS) ? vLinearMs : V_FLOOR_MS;
    float omega = vSteeringEffectiveMs * kappa; // [rad/s]
    out.omegaCmd = omega;

    // 4. Phân bổ tốc độ vi sai bánh xe (Differential Kinematics)
    // Khi vạch lệch phải (e > 0, omega > 0): Xe cần rẽ PHẢI
    // -> Bánh trái (ngoài) phải tăng tốc: v_left = v + diffSpeed
    // -> Bánh phải (trong) phải giảm tốc: v_right = v - diffSpeed
    float trackWidthM = (TRACK_WIDTH_MM * 0.001f);
    float diffSpeedMs = (omega * trackWidthM * 0.5f);

    float vLeftMs  = vLinearMs + diffSpeedMs;
    float vRightMs = vLinearMs - diffSpeedMs;

    float vLeftMmS  = vLeftMs * 1000.0f;
    float vRightMmS = vRightMs * 1000.0f;

    // 5. Giới hạn tốc độ bánh xe: nếu một bánh vượt v_wheel_max thì scale cả hai bánh theo cùng tỉ lệ
    // để BẢO TOÀN ĐỘ CONG KAPPA (không bị xén cắt méo mó góc lái!)
    float allowedWheelMax = fmaxf(maxWheelSpeedMmS, targetSpeedMmS + 200.0f);
    float maxReq = fmaxf(fabsf(vLeftMmS), fabsf(vRightMmS));
    if (maxReq > allowedWheelMax && maxReq > 0.001f) {
        float scale = allowedWheelMax / maxReq;
        vLeftMmS  *= scale;
        vRightMmS *= scale;
        targetSpeedMmS *= scale;
    }

    out.targetSpeedMmS = targetSpeedMmS;
    out.vLeftMmS  = vLeftMmS;
    out.vRightMmS = vRightMmS;

    return out;
}
