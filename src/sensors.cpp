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
// MUX Channel Switching & ADC Sampling (Stable & Clean)
// ----------------------------------------------------
static inline void setMux(uint8_t ch) {
    digitalWrite(PIN_MUX_S0, (ch & 0x01) ? HIGH : LOW);
    digitalWrite(PIN_MUX_S1, (ch & 0x02) ? HIGH : LOW);
    digitalWrite(PIN_MUX_S2, (ch & 0x04) ? HIGH : LOW);
}

uint16_t sensorsReadChannel(uint8_t ch) {
    setMux(ch);
    delayMicroseconds(MUX_SWITCH_DELAY_US);
    (void)analogRead(PIN_MUX_SIG); // Flush ADC S&H capacitance
    uint32_t s1 = analogRead(PIN_MUX_SIG);
    uint32_t s2 = analogRead(PIN_MUX_SIG);
    return (uint16_t)((s1 + s2) >> 1); // 2x Oversampling khử gai nhiễu
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

    // Xác định phân cực màu vạch (đen trên trắng hay trắng trên đen)
    int darkVotes = 0;
    int lightVotes = 0;
    for (int i = 0; i < 8; i++) {
        int span = sensorMax[i] - sensorMin[i];
        if (span >= 180) {
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

        if (span < 180) {
            sensorEnabled[i] = false;
            Serial.printf("  Ch %d: Min=%4u | Max=%4u | Span=%4u [TẮT - Không đủ tương phản (Cần >= 180)!]\n",
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

LineDetection sensorsProcess() {
    LineDetection det;
    det.positionMm     = 0.0f;
    det.positionNorm   = 0.0f;
    det.confidence     = 0.0f;
    det.lineDetected   = false;
    det.isExtrapolated = false;
    det.turn90         = 0;

    uint32_t startUs = micros();

    // 1. Đọc 8 kênh MUX ADC
    uint16_t raw[8];
    for (uint8_t i = 0; i < 8; i++) {
        raw[i] = sensorsReadChannel(i);
    }
    det.adcDurationUs = (uint32_t)(micros() - startUs);

    // 2. Chuẩn hóa analog S_i [0.0, 1.0]
    float S[8];
    float sMax = 0.0f;
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

        float val = 0.0f;
        if (lineIsDark[i]) {
            val = ((float)sensorMax[i] - (float)raw[i]) / (float)span;
        } else {
            val = ((float)raw[i] - (float)sensorMin[i]) / (float)span;
        }
        val = constrain(val, 0.0f, 1.0f);

        // Cắt sàn nhiễu 0.15
        if (val < 0.15f) val = 0.0f;
        else val = (val - 0.15f) / 0.85f;

        S[i] = val;
        lastNormValues[i] = (int)roundf(val * 1000.0f);

        if (val > sMax) sMax = val;
        sSum += val;
    }

    // 3. Tính trọng tâm vạch (Centroid nguyên thủy)
    if (sSum > 0.20f) {
        float num = 0.0f;
        float den = 0.0f;
        for (uint8_t i = 0; i < 8; i++) {
            if (S[i] > 0.0f) {
                num += S[i] * SENSOR_WEIGHTS[i];
                den += S[i];
            }
        }

        if (den > 0.01f) {
            float rawPos = num / den;
            det.lineDetected   = true;
            det.positionNorm   = rawPos + SENSOR_CENTER_TRIM;
            det.positionMm     = det.positionNorm * SENSOR_PITCH_MM;
            det.confidence     = constrain(sMax, 0.0f, 1.0f);
            det.isExtrapolated = false;

            if (det.positionNorm > 0.5f) lastGoodLineDir = 1;
            else if (det.positionNorm < -0.5f) lastGoodLineDir = -1;

            return det;
        }
    }

    // Ngoại suy khi vạch trượt hẳn ra mép ngoài
    if (S[7] > 0.20f && S[7] > S[0]) {
        det.lineDetected   = true;
        det.isExtrapolated = true;
        det.positionNorm   = SENSOR_EXTRAPOLATE_POS;
        det.positionMm     = SENSOR_EXTRAPOLATE_MM;
        det.confidence     = 0.5f;
        lastGoodLineDir    = 1;
        return det;
    } else if (S[0] > 0.20f && S[0] > S[7]) {
        det.lineDetected   = true;
        det.isExtrapolated = true;
        det.positionNorm   = -SENSOR_EXTRAPOLATE_POS;
        det.positionMm     = -SENSOR_EXTRAPOLATE_MM;
        det.confidence     = 0.5f;
        lastGoodLineDir    = -1;
        return det;
    }

    // Mất line
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



void sensorsPrintDebug(float currentError) {
    Serial.print("Sensors: [");
    for (uint8_t i = 0; i < 8; i++) {
        int norm = lastNormValues[i];
        if (!sensorEnabled[i]) {
            Serial.print("x"); // Disabled/dead sensor marker
        } else if (norm > 600) {
            Serial.print("#");
        } else if (norm > 180) {
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
        Serial.printf(" %u:%4u", i, r);
        if (i < 7) Serial.print(" |");
    }
    Serial.println(" ]");
}
