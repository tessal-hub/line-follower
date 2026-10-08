#include "sensors.h"
#include "config.h"
#include "display.h"
#include <Preferences.h>

static Preferences sensorPrefs;

static uint16_t sensorMin[8] = {300, 300, 300, 300, 300, 300, 300, 300};
static uint16_t sensorMax[8] = {3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000};
static uint32_t sensorAvg[8] = {0, 0, 0, 0, 0, 0, 0, 0};
static bool lineIsDark[8]    = {true, true, true, true, true, true, true, true};
static bool sensorEnabled[8] = {true, true, true, true, true, true, true, true}; // All enabled by default
static bool calibrated       = true; // Sensible analog defaults ready at startup
static int lastNormValues[8] = {0};
static int8_t lastGoodLineDir = 1;  // +1 = Line on Right, -1 = Line on Left

int8_t sensorsGetLastGoodDir() {
    return lastGoodLineDir;
}

void sensorsGetNormValues(int outValues[8]) {
    for (int i = 0; i < 8; i++) {
        outValues[i] = lastNormValues[i];
    }
}

// ----------------------------------------------------
// Ultra-Fast MUX Channel Switching via ESP32 GPIO Registers
// (Direct register write takes ~15ns vs 6000ns of 3x digitalWrite)
// ----------------------------------------------------
static inline void setMuxFast(uint8_t ch) {
    static const uint32_t MUX_ALL_MASK = (1UL << PIN_MUX_S0) | (1UL << PIN_MUX_S1) | (1UL << PIN_MUX_S2);
    static const uint32_t MUX_SET_MASKS[8] = {
        0,                                                                              // ch 0: 000
        (1UL << PIN_MUX_S0),                                                            // ch 1: 001
        (1UL << PIN_MUX_S1),                                                            // ch 2: 010
        (1UL << PIN_MUX_S0) | (1UL << PIN_MUX_S1),                                      // ch 3: 011
        (1UL << PIN_MUX_S2),                                                            // ch 4: 100
        (1UL << PIN_MUX_S0) | (1UL << PIN_MUX_S2),                                      // ch 5: 101
        (1UL << PIN_MUX_S1) | (1UL << PIN_MUX_S2),                                      // ch 6: 110
        (1UL << PIN_MUX_S0) | (1UL << PIN_MUX_S1) | (1UL << PIN_MUX_S2)               // ch 7: 111
    };
    uint32_t setMask = MUX_SET_MASKS[ch & 0x07];
    uint32_t clearMask = MUX_ALL_MASK & ~setMask;
    GPIO.out_w1tc = clearMask;
    GPIO.out_w1ts = setMask;
}

uint16_t sensorsReadChannel(uint8_t ch) {
    setMuxFast(ch);
    delayMicroseconds(MUX_SWITCH_DELAY_US);
    analogRead(PIN_MUX_SIG); // Flush ADC S&H capacitance to prevent channel ghosting
    long sum = 0;
    for (int s = 0; s < MUX_ADC_SAMPLES; s++) {
        sum += analogRead(PIN_MUX_SIG);
    }
    return (uint16_t)(sum / MUX_ADC_SAMPLES);
}

// ----------------------------------------------------
// NVS Flash Persistence (Preferences.h)
// ----------------------------------------------------
bool sensorsLoadCalibration() {
    if (!sensorPrefs.begin("sensors", true)) { // Read-only
        return false;
    }
    if (!sensorPrefs.isKey("valid") || !sensorPrefs.getBool("valid", false)) {
        sensorPrefs.end();
        return false;
    }

    uint16_t loadedMin[8] = {0};
    uint16_t loadedMax[8] = {0};
    uint8_t darkBytes[8] = {0};
    uint8_t enBytes[8] = {0};

    size_t readMin  = sensorPrefs.getBytes("min", loadedMin, sizeof(loadedMin));
    size_t readMax  = sensorPrefs.getBytes("max", loadedMax, sizeof(loadedMax));
    size_t readDark = sensorPrefs.getBytes("dark", darkBytes, sizeof(darkBytes));
    size_t readEn   = sensorPrefs.getBytes("en", enBytes, sizeof(enBytes));
    sensorPrefs.end();

    if (readMin != sizeof(sensorMin) || readMax != sizeof(sensorMax) ||
        readDark != sizeof(darkBytes) || readEn != sizeof(enBytes)) {
        return false;
    }

    for (int i = 0; i < 8; i++) {
        sensorMin[i]     = loadedMin[i];
        sensorMax[i]     = loadedMax[i];
        lineIsDark[i]    = (darkBytes[i] != 0);
        int span = (int)loadedMax[i] - (int)loadedMin[i];
        sensorEnabled[i] = (enBytes[i] != 0) || (span >= 150);
    }
    calibrated = true;

    Serial.println("\n[NVS FLASH] Đã nạp thành công thông số cảm biến từ Flash:");
    for (int i = 0; i < 8; i++) {
        int span = sensorMax[i] - sensorMin[i];
        Serial.printf("  Ch %d: Min=%4u | Max=%4u | Span=%4u | Line=%s [%s]\n",
            i, sensorMin[i], sensorMax[i], span,
            lineIsDark[i] ? "DARK" : "LIGHT",
            sensorEnabled[i] ? "OK" : "TẮT");
    }
    Serial.println();
    return true;
}

void sensorsSaveCalibration() {
    if (!sensorPrefs.begin("sensors", false)) { // Read-write
        Serial.println("[NVS FLASH] LỖI: Không thể mở Flash để lưu!");
        return;
    }
    uint8_t darkBytes[8];
    uint8_t enBytes[8];
    for (int i = 0; i < 8; i++) {
        darkBytes[i] = lineIsDark[i] ? 1 : 0;
        enBytes[i]   = sensorEnabled[i] ? 1 : 0;
    }
    sensorPrefs.putBytes("min", sensorMin, sizeof(sensorMin));
    sensorPrefs.putBytes("max", sensorMax, sizeof(sensorMax));
    sensorPrefs.putBytes("dark", darkBytes, sizeof(darkBytes));
    sensorPrefs.putBytes("en", enBytes, sizeof(enBytes));
    sensorPrefs.putBool("valid", true);
    sensorPrefs.end();

    Serial.println("[NVS FLASH] Đã lưu thông số cảm biến vào Flash thành công!");
}

void sensorsInit() {
    pinMode(PIN_MUX_S0, OUTPUT);
    pinMode(PIN_MUX_S1, OUTPUT);
    pinMode(PIN_MUX_S2, OUTPUT);
    pinMode(PIN_MUX_SIG, INPUT);

    analogReadResolution(12);
    analogSetPinAttenuation(PIN_MUX_SIG, ADC_11db); // Full 0V - ~3.1V dynamic range

    // Try loading saved calibration from ESP32 Flash
    if (!sensorsLoadCalibration()) {
        Serial.println("[NVS FLASH] Chưa có dữ liệu calib trong Flash. Dùng thông số mặc định.");
        Serial.println("            (Bấm SW1 để quét 10s và lưu vĩnh viễn vào Flash!)");
    }
}

void sensorsCalibrate(uint32_t durationMs) {
    Serial.println("\n==========================================");
    Serial.printf("[CALIB] Bắt đầu Calibrate trong %u giây...\n", durationMs / 1000);
    Serial.println("  Trượt đều toàn bộ xe qua lại vạch đen để cả 8 mắt đều quét qua!");
    Serial.println("==========================================");
    delay(800);

    for (int i = 0; i < 8; i++) {
        sensorMin[i] = 4095;
        sensorMax[i] = 0;
        sensorAvg[i] = 0;
    }

    uint32_t sampleCount = 0;
    uint32_t startMs = millis();
    uint32_t lastDotMs = 0;
    while (millis() - startMs < durationMs) {
        uint32_t elapsed = millis() - startMs;
        displaySetCalibProgress(elapsed, durationMs);
        for (uint8_t i = 0; i < 8; i++) {
            uint16_t val = sensorsReadChannel(i);
            if (val < sensorMin[i]) sensorMin[i] = val;
            if (val > sensorMax[i]) sensorMax[i] = val;
            sensorAvg[i] += val;
        }
        sampleCount++;
        if (millis() - lastDotMs > 500) {
            lastDotMs = millis();
            Serial.print(".");
        }
        delay(2);
    }
    Serial.println(" XONG!\n");

    if (sampleCount == 0) sampleCount = 1;

    // Determine majority line polarity across sensors with valid contrast
    int darkVotes = 0;
    int lightVotes = 0;
    for (int i = 0; i < 8; i++) {
        int span = sensorMax[i] - sensorMin[i];
        if (span >= 500) {
            uint32_t mean = sensorAvg[i] / sampleCount;
            uint16_t midpoint = (sensorMin[i] + sensorMax[i]) / 2;
            if (mean > midpoint) darkVotes++;
            else lightVotes++;
        }
    }
    bool uniformLineIsDark = (darkVotes >= lightVotes);

    Serial.println("[CALIB] Kết quả Calibrate:");
    for (int i = 0; i < 8; i++) {
        int span = sensorMax[i] - sensorMin[i];
        lineIsDark[i] = uniformLineIsDark;

        if (span < 150) {
            sensorEnabled[i] = false;
            Serial.printf("  Ch %d: Min=%4u | Max=%4u | Span=%4u [TẮT - Không có tương phản!]\n",
                i, sensorMin[i], sensorMax[i], span);
        } else {
            sensorEnabled[i] = true;
            Serial.printf("  Ch %d: Min=%4u | Max=%4u | Span=%4u | Line=%s [OK]\n",
                i, sensorMin[i], sensorMax[i], span, uniformLineIsDark ? "DARK" : "LIGHT");
        }
    }
    calibrated = true;
    sensorsSaveCalibration();
    displayShowMessage("CALIBRATION", "SAVED TO FLASH!", 1800);
    Serial.println("==========================================\n");
}

bool sensorsIsCalibrated() {
    return calibrated;
}

static inline uint16_t sensorsReadFast(uint8_t ch) {
    setMuxFast(ch);
    delayMicroseconds(10); // 10us settle time đảm bảo nạp xả tụ sạch sẽ
    return (uint16_t)analogRead(PIN_MUX_SIG);
}

uint32_t sensorsBenchmarkLatency() {
    uint32_t t0 = micros();
    for (uint8_t i = 0; i < 8; i++) {
        sensorsReadFast(i);
    }
    return (micros() - t0);
}

LineDetection sensorsProcess() {
    LineDetection det;
    det.positionMm     = 0.0f;
    det.positionNorm   = 0.0f;
    det.confidence     = 0.0f;
    det.lineDetected   = false;
    det.isExtrapolated = false;
    det.turn90         = 0;

    uint32_t startUs = micros();

    // 1. Đọc nhanh 8 kênh MUX ADC (settle 10us)
    uint16_t raw[8];
    for (uint8_t i = 0; i < 8; i++) {
        raw[i] = sensorsReadFast(i);
    }
    det.adcDurationUs = (uint32_t)(micros() - startUs);

    // 2. Chuẩn hóa analog S_i liên tục [0.0, 1.0] theo đúng phân cực lineIsDark
    float S[8];
    float sMax = 0.0f;
    float sMin = 1.0f;
    float sSum = 0.0f;

    for (uint8_t i = 0; i < 8; i++) {
        if (!sensorEnabled[i]) {
            S[i] = 0.0f;
            lastNormValues[i] = 0;
            continue;
        }
        int span = (int)sensorMax[i] - (int)sensorMin[i];
        if (span < 150) {
            S[i] = 0.0f;
            lastNormValues[i] = 0;
            continue;
        }

        // Bù đúng phân cực màu: Vạch đen trên nền trắng thì raw nhỏ -> S_i cao (1.0)
        float val = 0.0f;
        if (lineIsDark[i]) {
            val = ((float)sensorMax[i] - (float)raw[i]) / (float)span;
        } else {
            val = ((float)raw[i] - (float)sensorMin[i]) / (float)span;
        }
        val = constrain(val, 0.0f, 1.0f);
        S[i] = val;
        lastNormValues[i] = (int)(val * 1000.0f); // Tương thích hiển thị OLED & Telemetry

        if (val > sMax) sMax = val;
        if (val < sMin) sMin = val;
        sSum += val;
    }

    // 3. Phát hiện vạch ngang góc vuông 90° (T-junction / L-corner)
    int rc = 0, lc = 0;
    for (int i = 4; i < 8; i++) if (S[i] > 0.28f) rc++;
    for (int i = 0; i < 4; i++) if (S[i] > 0.28f) lc++;

    int totalActive = rc + lc;
    if (totalActive >= 6) {
        det.turn90 = 0; // Giao lộ chữ thập hoặc vạch đích rộng
    } else if (rc >= 3 && lc <= 1) {
        det.turn90 = 1; // Góc vuông phải
    } else if (lc >= 3 && rc <= 1) {
        det.turn90 = -1; // Góc vuông trái
    }

    // 4. Tính trọng tâm Centroid: Position = Σ(S_i * w_i) / Σ(S_i)
    const float S_FLOOR = 0.15f; // Trừ sàn liên tục 15% để khử nhiễu nền
    float num = 0.0f;
    float den = 0.0f;
    for (uint8_t i = 0; i < 8; i++) {
        if (S[i] > S_FLOOR) {
            float active = S[i] - S_FLOOR;
            num += active * SENSOR_WEIGHTS[i];
            den += active;
        }
    }

    if (den > 0.10f) {
        float rawPos = num / den;
        det.lineDetected = true;

        // Nếu vạch nằm sát mép ngoài cùng và đang trượt ra biên: ngoại suy mượt mà
        if (rawPos > 3.0f && S[7] > 0.30f && S[7] >= S[6]) {
            det.isExtrapolated = true;
            det.positionNorm   = SENSOR_EXTRAPOLATE_POS; // +4.25
            det.positionMm     = SENSOR_EXTRAPOLATE_MM;  // ~ +50.39 mm
            lastGoodLineDir    = 1;
        } else if (rawPos < -3.0f && S[0] > 0.30f && S[0] >= S[1]) {
            det.isExtrapolated = true;
            det.positionNorm   = -SENSOR_EXTRAPOLATE_POS; // -4.25
            det.positionMm     = -SENSOR_EXTRAPOLATE_MM;  // ~ -50.39 mm
            lastGoodLineDir    = -1;
        } else {
            det.isExtrapolated = false;
            det.positionNorm   = rawPos + SENSOR_CENTER_TRIM;
            det.positionMm     = det.positionNorm * SENSOR_PITCH_MM;
            if (det.positionNorm > 0.80f) {
                lastGoodLineDir = 1;
            } else if (det.positionNorm < -0.80f) {
                lastGoodLineDir = -1;
            }
        }

        // Tính Confidence ∈ [0.0, 1.0]:
        float cPeak     = constrain((sMax - 0.20f) / 0.55f, 0.0f, 1.0f);
        float cContrast = constrain((sMax - sMin - 0.15f) / 0.50f, 0.0f, 1.0f);
        float cSum      = 1.0f - constrain(fabsf(sSum - 1.8f) / 2.5f, 0.0f, 1.0f);

        det.confidence = constrain(cPeak * cContrast * (0.65f + 0.35f * cSum), 0.0f, 1.0f);
        return det;
    }

    // 5. Ngoại suy khẩn cấp khi line đã trượt ra ngoài thanh (den <= 0.10f):
    if (S[7] > 0.20f && S[0] < 0.15f && S[1] < 0.15f && S[2] < 0.15f) {
        det.lineDetected   = true;
        det.isExtrapolated = true;
        det.positionNorm   = SENSOR_EXTRAPOLATE_POS;
        det.positionMm     = SENSOR_EXTRAPOLATE_MM;
        det.confidence     = constrain(S[7] * 0.70f, 0.20f, 0.50f);
        lastGoodLineDir    = 1;
        return det;
    } else if (S[0] > 0.20f && S[7] < 0.15f && S[6] < 0.15f && S[5] < 0.15f) {
        det.lineDetected   = true;
        det.isExtrapolated = true;
        det.positionNorm   = -SENSOR_EXTRAPOLATE_POS;
        det.positionMm     = -SENSOR_EXTRAPOLATE_MM;
        det.confidence     = constrain(S[0] * 0.70f, 0.20f, 0.50f);
        lastGoodLineDir    = -1;
        return det;
    }

    // Mất line:
    det.lineDetected   = false;
    det.isExtrapolated = false;
    det.confidence     = 0.0f;
    det.positionNorm   = 0.0f;
    det.positionMm     = 0.0f;
    return det;
}

float sensorsGetError(float lastKnownError, bool &lineDetected, int &turn90) {
    LineDetection det = sensorsProcess();
    lineDetected = det.lineDetected;
    turn90 = det.turn90;
    if (det.lineDetected) {
        return det.positionMm;
    }
    return lastKnownError;
}

bool sensorsIsFlankActive() {
    bool leftFlank  = (sensorEnabled[0] && lastNormValues[0] > 300) || (sensorEnabled[1] && lastNormValues[1] > 380);
    bool rightFlank = (sensorEnabled[7] && lastNormValues[7] > 300) || (sensorEnabled[6] && lastNormValues[6] > 380);
    return (leftFlank || rightFlank);
}

static inline int getNormalized(uint8_t i, uint16_t raw) {
    if (!sensorEnabled[i]) return 0;
    int span = (int)sensorMax[i] - (int)sensorMin[i];
    if (span < 150) return 0;
    int norm = 0;
    if (lineIsDark[i]) {
        norm = (int)(((long)(sensorMax[i] - raw) * 1000) / span);
    } else {
        norm = (int)(((long)(raw - sensorMin[i]) * 1000) / span);
    }
    return constrain(norm, 0, 1000);
}

int sensorsGetCenterLine() {
    uint16_t raw3 = sensorsReadChannel(3);
    uint16_t raw4 = sensorsReadChannel(4);
    int n3 = getNormalized(3, raw3);
    int n4 = getNormalized(4, raw4);
    return (n3 > n4) ? n3 : n4;
}

bool sensorsHasReachedCenter(bool isTurningRight) {
    uint16_t raw3 = sensorsReadChannel(3);
    uint16_t raw4 = sensorsReadChannel(4);
    int n3 = getNormalized(3, raw3);
    int n4 = getNormalized(4, raw4);

    // Pivot is complete when either center sensor confirms the new line
    return (n3 > 260 || n4 > 260);
}

void sensorsPrintDebug(float currentError) {
    Serial.print("Sensors: [");
    for (uint8_t i = 0; i < 8; i++) {
        int norm = lastNormValues[i];
        if (!sensorEnabled[i]) {
            Serial.print("x"); // Disabled/dead sensor marker
        } else if (norm > 700) {
            Serial.print("#");
        } else if (norm > 250) {
            Serial.print("+");
        } else {
            Serial.print(".");
        }
        if (i < 7) Serial.print(" ");
    }
    Serial.printf("] Err:%+5.2f (Ch3:%3d Ch4:%3d)\n", currentError, lastNormValues[3], lastNormValues[4]);
}

void sensorsPrintRaw() {
    Serial.print("  Raw ADC: [");
    for (uint8_t i = 0; i < 8; i++) {
        uint16_t r = sensorsReadChannel(i);
        Serial.printf("%5u", r);
        if (i < 7) Serial.print(" ");
    }
    Serial.println("]");
}
