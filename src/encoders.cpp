#include "encoders.h"
#include "config.h"

static volatile int32_t leftTicks = 0;
static volatile int32_t rightTicks = 0;

static int32_t lastLeftTicks = 0;
static int32_t lastRightTicks = 0;

static const int N = 5;
static uint32_t histT[N] = {0};
static int32_t histL[N] = {0};
static int32_t histR[N] = {0};
static int hIdx = 0;

static float leftSpeedMmS = 0.0f;
static float rightSpeedMmS = 0.0f;

// Distance per single rising-edge tick (mm)
// (PI * 68mm) / 68.75 ticks = ~3.107 mm per tick
static const float MM_PER_TICK = (3.14159265f * WHEEL_DIAMETER_MM) / ENCODER_TICKS_PER_REV;

void IRAM_ATTR isrLeftEncoder() {
    if (digitalRead(PIN_ENC_LEFT_B)) {
        leftTicks++;
    } else {
        leftTicks--;
    }
}

void IRAM_ATTR isrRightEncoder() {
    // Note: If right motor is mirrored, swap +/- if needed
    if (digitalRead(PIN_ENC_RIGHT_B)) {
        rightTicks++;
    } else {
        rightTicks--;
    }
}

void encodersInit() {
    pinMode(PIN_ENC_LEFT_A, INPUT_PULLUP);
    pinMode(PIN_ENC_LEFT_B, INPUT_PULLUP);

    // GPIO 34 & 35 are input-only (must have external pull-ups)
    pinMode(PIN_ENC_RIGHT_A, INPUT);
    pinMode(PIN_ENC_RIGHT_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(PIN_ENC_LEFT_A), isrLeftEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_RIGHT_A), isrRightEncoder, RISING);

    encodersResetTicks();
}

int32_t encodersGetLeftTicks() {
    return leftTicks;
}

int32_t encodersGetRightTicks() {
    return rightTicks;
}

void encodersResetTicks() {
    noInterrupts();
    leftTicks = 0;
    rightTicks = 0;
    interrupts();
    lastLeftTicks = 0;
    lastRightTicks = 0;
    for (int i = 0; i < N; i++) {
        histT[i] = 0;
        histL[i] = 0;
        histR[i] = 0;
    }
    hIdx = 0;
    leftSpeedMmS = 0.0f;
    rightSpeedMmS = 0.0f;
}

float encodersGetLeftDistanceMm() {
    return (float)leftTicks * MM_PER_TICK;
}

float encodersGetRightDistanceMm() {
    return (float)rightTicks * MM_PER_TICK;
}

void encodersUpdateSpeed() {
    uint32_t t = micros();
    int32_t cl = leftTicks;
    int32_t cr = rightTicks;
    uint32_t t0 = histT[hIdx];
    int32_t l0 = histL[hIdx];
    int32_t r0 = histR[hIdx];

    histT[hIdx] = t;
    histL[hIdx] = cl;
    histR[hIdx] = cr;
    hIdx = (hIdx + 1) % N;

    if (t0 == 0) return;
    float dt = (float)(t - t0) * 1e-6f;
    if (dt < 0.005f) return;

    leftSpeedMmS  = (float)(cl - l0) * MM_PER_TICK / dt;
    rightSpeedMmS = (float)(cr - r0) * MM_PER_TICK / dt;
}

float encodersGetLeftSpeedMmS() {
    return leftSpeedMmS;
}

float encodersGetRightSpeedMmS() {
    return rightSpeedMmS;
}


