#include "track_profile.h"
#include <math.h>

static TrackPoint trackMap[MAX_TRACK_POINTS];
static uint16_t trackPointCount = 0;
static bool isRecording = false;
static bool hasProfile = false;
static float lastRecordedSM = -1.0f;

void trackProfileInit() {
    trackProfileReset();
}

void trackProfileReset() {
    trackPointCount = 0;
    isRecording = false;
    hasProfile = false;
    lastRecordedSM = -1.0f;
    for (int i = 0; i < MAX_TRACK_POINTS; i++) {
        trackMap[i].kappa = 0.0f;
        trackMap[i].vMax = 1200.0f;
        trackMap[i].vPlan = 1200.0f;
    }
}

void trackProfileStartRecording() {
    trackProfileReset();
    isRecording = true;
}

void trackProfileStopRecording() {
    isRecording = false;
}

bool trackProfileIsRecorded() {
    return hasProfile;
}

void trackProfileRecordStep(float distSM, float measuredKappa, float confidence) {
    if (!isRecording || confidence < 0.40f) return;

    if (trackPointCount >= MAX_TRACK_POINTS) {
        isRecording = false;
        return;
    }

    // Chỉ ghi điểm mới khi đã đi qua bước quãng đường delta_s = 50mm
    if (distSM - lastRecordedSM >= TRACK_DS_M || lastRecordedSM < 0.0f) {
        lastRecordedSM = distSM;
        trackMap[trackPointCount].kappa = measuredKappa;
        trackPointCount++;
    }
}

void trackProfileGenerateProfile(const PlannerConfig &cfg) {
    if (trackPointCount < 10) {
        hasProfile = false;
        return;
    }

    float dsM = TRACK_DS_M;

    // 1. Tính v_max(s) từ giới hạn gia tốc ngang: v_max = sqrt(a_lat / |kappa|)
    for (uint16_t i = 0; i < trackPointCount; i++) {
        float absKappa = fabsf(trackMap[i].kappa);
        if (absKappa < 0.05f) { // Bán kính cong > 20 mét (đoạn thẳng)
            trackMap[i].vMax = cfg.vMaxMotor;
        } else {
            // v_max [m/s] = sqrt(a_lat / kappa)
            float vMaxMs = sqrtf(cfg.aLatMax / absKappa);
            float vMaxMmS = vMaxMs * 1000.0f;

            // v_min chỉ là sàn bảo vệ, không được nâng v_max vượt trần động cơ
            vMaxMmS = fminf(vMaxMmS, cfg.vMaxMotor);
            vMaxMmS = fmaxf(vMaxMmS, cfg.vMinCorner);
            trackMap[i].vMax = vMaxMmS;
        }
    }

    // 2. Forward Pass: Giới hạn bởi khả năng tăng tốc của động cơ
    // v[i+1] = min(sqrt(v[i]^2 + 2 * a_accel * ds), v_max[i+1])
    float vFwd[MAX_TRACK_POINTS];
    vFwd[0] = trackMap[0].vMax;

    for (uint16_t i = 0; i < trackPointCount - 1; i++) {
        float vCurrentMs = (vFwd[i] * 0.001f);
        // Gia tốc giảm theo vận tốc do Back-EMF: a_accel = a0 * (1 - v / v_max_motor)
        float factor = 1.0f - (vFwd[i] / cfg.vMaxMotor);
        factor = constrain(factor, 0.20f, 1.0f);
        float aAccelMs2 = cfg.aAccelMax * factor;

        float vNextMs = sqrtf(vCurrentMs * vCurrentMs + 2.0f * aAccelMs2 * dsM);
        float vNextMmS = vNextMs * 1000.0f;

        vFwd[i + 1] = fminf(vNextMmS, trackMap[i + 1].vMax);
    }

    // 3. Backward Pass: Giới hạn bởi khả năng phanh trước khúc cua
    // v[i] = min(v_forward[i], sqrt(v[i+1]^2 + 2 * a_brake * ds))
    trackMap[trackPointCount - 1].vPlan = vFwd[trackPointCount - 1];

    for (int i = (int)trackPointCount - 2; i >= 0; i--) {
        float vNextPlanMs = (trackMap[i + 1].vPlan * 0.001f);
        float vAllowedMs = sqrtf(vNextPlanMs * vNextPlanMs + 2.0f * cfg.aBrakeMax * dsM);
        float vAllowedMmS = vAllowedMs * 1000.0f;

        trackMap[i].vPlan = fminf(vFwd[i], vAllowedMmS);
    }

    // 4. Dịch điểm phanh sớm hơn một khoảng delta_s_lag = v * t_lag
    int shiftSteps = (int)roundf((cfg.vMaxMotor * 0.001f * cfg.tLagS) / dsM);
    if (shiftSteps > 0 && shiftSteps < 10) {
        for (uint16_t i = 0; i < trackPointCount - shiftSteps; i++) {
            trackMap[i].vPlan = fminf(trackMap[i].vPlan, trackMap[i + shiftSteps].vPlan);
        }
    }

    hasProfile = true;
}

float trackProfileGetTargetSpeed(float sM) {
    if (!hasProfile || trackPointCount == 0 || sM < 0.0f) {
        return -1.0f; // Chưa có profile, dùng reactive fallback
    }

    int idx = (int)(sM / TRACK_DS_M);
    if (idx >= trackPointCount) {
        idx = trackPointCount - 1;
    }
    return trackMap[idx].vPlan;
}

float trackProfileComputeReactiveFallback(
    float errorMm,
    float dErrorMmS,
    float confidence,
    float baseSpeedMmS,
    float cornerSpeedMmS,
    float currentSpeedMmS
) {
    // 1. Phản ứng theo độ lệch vị trí (eNorm bắt đầu từ 2mm lệch, đạt 1.0 ở 14mm)
    float absEMm = fabsf(errorMm);
    float eNorm = constrain((absEMm - 2.0f) / 12.0f, 0.0f, 1.0f);

    // 2. Phản ứng theo tốc độ đổi hướng (de/dt)
    float absDe = fabsf(dErrorMmS);
    float deNorm = constrain(absDe / 350.0f, 0.0f, 1.0f);

    float intensity = constrain(0.70f * eNorm + 0.45f * deNorm, 0.0f, 1.0f);

    // Nếu độ tin cậy thấp (sắp mất vạch), ghìm tốc độ tức thì
    if (confidence < 0.50f) {
        float confPen = (0.50f - confidence) / 0.50f;
        intensity = fmaxf(intensity, confPen);
    }

    float target = baseSpeedMmS - (baseSpeedMmS - cornerSpeedMmS) * intensity;
    return target;
}

bool trackProfileCheckSafetyOverride(
    float errorMm,
    float confidence,
    float eLimitMm,
    float confLimit
) {
    // Kích hoạt Safety Override khi sai số vượt giới hạn an toàn hoặc mất độ tin cậy
    if (fabsf(errorMm) > eLimitMm || confidence < confLimit) {
        return true;
    }
    return false;
}
