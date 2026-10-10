#include "encoders.h"
#include "config.h"

static volatile int32_t leftTicks = 0;
static volatile int32_t rightTicks = 0;

static volatile uint8_t lastLeftState = 0;
static volatile uint8_t lastRightState = 0;

// Quadrature 4x Gray-code state transition lookup table:
// Index: (old_state << 2) | new_state (where state = (B << 1) | A)
// Return: +1 = forward, -1 = backward, 0 = no change or invalid glitch
static const int8_t QUAD_TABLE[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

static const int N = 5;
static uint32_t histT[N] = {0};
static int32_t histL[N] = {0};
static int32_t histR[N] = {0};
static int hIdx = 0;

static float leftSpeedMmS = 0.0f;
static float rightSpeedMmS = 0.0f;

// Distance per 4x quadrature tick MM_PER_TICK is defined in config.h (~0.7768 mm per tick)

static volatile uint32_t lastLeftEdgeUs = 0;
static volatile uint32_t lastRightEdgeUs = 0;

// ISR chạy cực nhanh bằng cách đọc trực tiếp thanh ghi GPIO.in1.data của ESP32 (~15ns)
// GPIO 32 -> bit 0, GPIO 33 -> bit 1, GPIO 34 -> bit 2, GPIO 35 -> bit 3
// Lọc nhiễu chặt chẽ: Chỉ cập nhật trạng thái khi bước chuyển Gray hợp lệ (step != 0).
// Bỏ qua và triệt tiêu 100% xung nhiễu nhọn từ PWM động cơ nhảy 2 bit cùng lúc!
void IRAM_ATTR isrLeftEncoder() {
    uint32_t gpio = GPIO.in1.data;
    uint8_t curr = gpio & 0x03; // Bits 0 and 1 (GPIO 32, 33)
    if (curr == lastLeftState) return;

    int8_t step = QUAD_TABLE[(lastLeftState << 2) | curr];
    if (step != 0) {
#if INVERT_LEFT_ENCODER
        leftTicks -= step;
#else
        leftTicks += step;
#endif
        lastLeftEdgeUs = micros();
        lastLeftState = curr;
    }
}

void IRAM_ATTR isrRightEncoder() {
    uint32_t gpio = GPIO.in1.data;
    uint8_t curr = (gpio >> 2) & 0x03; // Bits 2 and 3 (GPIO 34, 35)
    if (curr == lastRightState) return;

    int8_t step = QUAD_TABLE[(lastRightState << 2) | curr];
    if (step != 0) {
#if INVERT_RIGHT_ENCODER
        rightTicks -= step;
#else
        rightTicks += step;
#endif
        lastRightEdgeUs = micros();
        lastRightState = curr;
    }
}

void encodersInit() {
    pinMode(PIN_ENC_LEFT_A, INPUT_PULLUP);
    pinMode(PIN_ENC_LEFT_B, INPUT_PULLUP);

    // GPIO 34 & 35 are input-only pins on ESP32 (require external pull-ups)
    pinMode(PIN_ENC_RIGHT_A, INPUT);
    pinMode(PIN_ENC_RIGHT_B, INPUT);

    // Khởi tạo trạng thái ban đầu của encoder
    uint32_t gpio = GPIO.in1.data;
    lastLeftState = gpio & 0x03;
    lastRightState = (gpio >> 2) & 0x03;

    // Ngắt 4x giải mã quadrature: bắt cả hai sườn (CHANGE) trên cả 2 kênh A & B
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_LEFT_A), isrLeftEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_LEFT_B), isrLeftEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_RIGHT_A), isrRightEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_RIGHT_B), isrRightEncoder, CHANGE);

    encodersResetTicks();
}

int32_t encodersGetLeftTicks() {
    noInterrupts();
    int32_t t = leftTicks;
    interrupts();
    return t;
}

int32_t encodersGetRightTicks() {
    noInterrupts();
    int32_t t = rightTicks;
    interrupts();
    return t;
}

void encodersResetTicks() {
    noInterrupts();
    leftTicks = 0;
    rightTicks = 0;
    uint32_t gpio = GPIO.in1.data;
    lastLeftState = gpio & 0x03;
    lastRightState = (gpio >> 2) & 0x03;
    interrupts();

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
    return (float)encodersGetLeftTicks() * MM_PER_TICK;
}

float encodersGetRightDistanceMm() {
    return (float)encodersGetRightTicks() * MM_PER_TICK;
}

void encodersUpdateSpeed() {
    uint32_t t = micros();
    noInterrupts();
    int32_t cl = leftTicks;
    int32_t cr = rightTicks;
    uint32_t lastLEdge = lastLeftEdgeUs;
    uint32_t lastREdge = lastRightEdgeUs;
    interrupts();

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

    float rawSpeedL = (float)(cl - l0) * MM_PER_TICK / dt;
    float rawSpeedR = (float)(cr - r0) * MM_PER_TICK / dt;

    // Nếu không có xung cạnh nào trong 40ms (> 0.04s, tương đương tốc độ < 20 mm/s),
    // kết luận bánh xe đã dừng hẳn hoặc đổi chiều, xả tốc độ về 0 ngay lập tức:
    if (t - lastLEdge > 40000) rawSpeedL = 0.0f;
    if (t - lastREdge > 40000) rawSpeedR = 0.0f;

    // Bộ lọc thông thấp EMA (alpha = 0.45): triệt tiêu hoàn toàn rung lượng tử hóa
    leftSpeedMmS  = 0.45f * rawSpeedL + 0.55f * leftSpeedMmS;
    rightSpeedMmS = 0.45f * rawSpeedR + 0.55f * rightSpeedMmS;
}

float encodersGetLeftSpeedMmS() {
    return leftSpeedMmS;
}

float encodersGetRightSpeedMmS() {
    return rightSpeedMmS;
}


