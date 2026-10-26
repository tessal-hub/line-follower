#pragma once
#include <Arduino.h>

struct LineDetection {
    float positionMm;      // Vị trí vạch e so với tâm xe [mm], quy đổi từ pitch thực 11.857 mm
    float positionNorm;    // Vị trí chuẩn hóa [-3.5, +3.5]
    float confidence;      // Độ tin cậy vạch [0.0, 1.0]
    bool  lineDetected;    // true nếu phát hiện vạch
    bool  isExtrapolated;  // true nếu line nằm ngoài thanh mắt (đang ngoại suy)
    int   turn90;          // 0 (tương thích API)
    uint32_t adcDurationUs;// Thời gian thực tế đọc 8 kênh MUX ADC (us)
};

void sensorsInit();
void sensorsCalibrate(uint32_t durationMs);
bool sensorsIsCalibrated();
bool sensorsLoadCalibration();
void sensorsSaveCalibration();

// Hàm xử lý chính: đọc MUX ADC, chuẩn hóa analog, tính vị trí mm, confidence, ngoại suy
LineDetection sensorsProcess();

// API tuong thich cho che do IDLE & Debug
float sensorsGetError(float lastKnownError, bool &lineDetected, int &turn90);
uint16_t sensorsReadChannel(uint8_t ch);
void sensorsPrintDebug(float currentError);
void sensorsPrintRaw();
int8_t sensorsGetLastGoodDir();
void sensorsGetNormValues(int outValues[8]);
