#include "sensor.h"
#include "config.h"
#include "params.h"
#include "driver/adc.h"
#include <Preferences.h>

unsigned int sensorValue[8] = {0};
unsigned int sensorPID[8] = {0};
unsigned int black_value[8] = {0};
unsigned int white_value[8] = {0};
unsigned int compare_value[8] = {0};

unsigned char sensorBits = 0;
long posPID = 0;
long lastPos = 0;
int  RememberLine = 0;
bool wasLineLost = false;
float dFiltered = 0.0f;
bool cornerCrossbarDetected = false;
bool finishLineDetected = false;

static int centerClearCnt = 0;
static int finishLineTicks = 0;

static inline unsigned int calcCompareThreshold(unsigned int b, unsigned int w) {
  if (w <= b) return b;
  return b + ((w - b) * 55U) / 100U;
}

void sensorsInit() {
  pinMode(PIN_MUX_S0, OUTPUT);
  pinMode(PIN_MUX_S1, OUTPUT);
  pinMode(PIN_MUX_S2, OUTPUT);
  pinMode(PIN_MUX_SIG, INPUT);

  // Cấu hình ADC1 trực tiếp qua ESP-IDF: adc1_get_raw() nhanh hơn analogRead()
  // (bỏ qua lookup pin->channel và init check mỗi lần gọi). GPIO36 = ADC1_CH0.
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);

  readCalib();
  resetSensorTracking();
}

void resetSensorTracking() {
  RememberLine = 0;
  centerClearCnt = 0;
  posPID = 0;
  lastPos = 0;
  wasLineLost = false;
  dFiltered = 0.0f;
  cornerCrossbarDetected = false;
  finishLineDetected = false;
  finishLineTicks = 0;
}

unsigned char sensorMask(unsigned char mask) {
  return (sensorBits & mask);
}

int getSensorActiveCount() {
  return __builtin_popcount((unsigned int)sensorBits);
}

// Chuyển kênh MUX bằng 1 lần ghi thanh ghi GPIO trực tiếp (~1 chu kỳ CPU)
// thay vì 3 lần digitalWrite (~3-6µs). S0/S1/S2 = GPIO 16/17/23 đều < 32 nên
// nằm cùng thanh ghi GPIO.out. An toàn: chỉ gọi trong main-loop context.
#define MUX_PIN_MASK  ((1UL << PIN_MUX_S0) | (1UL << PIN_MUX_S1) | (1UL << PIN_MUX_S2))
static const uint32_t MUX_SEL_MASK[8] = {
  0UL,
  (1UL << PIN_MUX_S0),
  (1UL << PIN_MUX_S1),
  (1UL << PIN_MUX_S0) | (1UL << PIN_MUX_S1),
  (1UL << PIN_MUX_S2),
  (1UL << PIN_MUX_S0) | (1UL << PIN_MUX_S2),
  (1UL << PIN_MUX_S1) | (1UL << PIN_MUX_S2),
  MUX_PIN_MASK
};

void read_sensor() {
  unsigned char temp = 0;

  // 1. Quét 8 kênh qua MUX CD74HC4051
  for (int j = 0; j < 8; j++) {
    GPIO.out = (GPIO.out & ~MUX_PIN_MASK) | MUX_SEL_MASK[j];
    delayMicroseconds(MUX_SWITCH_DELAY_US);
    int r = adc1_get_raw(ADC1_CHANNEL_0);
    sensorValue[j] = 4095 - r;
  }

  // 2. Chuẩn hóa giá trị cho PID [0..3000] và sinh bitmask
  for (int j = 0; j < 8; j++) {
    unsigned int b = black_value[j];
    unsigned int w = white_value[j];
    if (w <= b) w = b + 1;

    unsigned int v = sensorValue[j];
    if (v < b) v = b;
    if (v > w) v = w;
    unsigned int mapped = map(v, b, w, 0, 3000);
    sensorPID[j] = (mapped < 200) ? 0 : mapped;

    temp = temp << 1;
    if (sensorValue[j] > compare_value[j]) {
      temp |= 0x01;
    }
  }
  sensorBits = temp;

  int activeCount = getSensorActiveCount();
  cornerCrossbarDetected = false;

  // 3. Nhận diện vạch đích / dừng (Finish Line: 7..8 mắt đen cùng lúc)
#if FINISH_LINE_ENABLE
  if (activeCount >= 7) {
    if (++finishLineTicks >= FINISH_LINE_MIN_TICKS) {
      finishLineDetected = true;
    }
  } else {
    finishLineTicks = 0;
  }
#endif

  // 4. Nhận diện vạch ngang góc vuông 90° (3..6 mắt một bên cùng đen)
  if (activeCount >= 3 && activeCount <= 6) {
    // Bên phải: Ch5,6,7 đen và 2 mắt rìa trái trắng
    if ((sensorBits & 0b00000111) == 0b00000111 && (sensorBits & 0b11000000) == 0) {
      RememberLine = 1;
      centerClearCnt = 0;
      cornerCrossbarDetected = true;
    }
    // Bên trái: Ch0,1,2 đen và 2 mắt rìa phải trắng
    else if ((sensorBits & 0b11100000) == 0b11100000 && (sensorBits & 0b00000011) == 0) {
      RememberLine = -1;
      centerClearCnt = 0;
      cornerCrossbarDetected = true;
    }
  }

  // 5. Cập nhật RememberLine ở chế độ bám mép bình thường
  if (!cornerCrossbarDetected) {
    if ((sensorBits & 0b00000011) != 0 && (sensorBits & 0b11000000) == 0) {
      RememberLine = 1;
      centerClearCnt = 0;
    } else if ((sensorBits & 0b11000000) != 0 && (sensorBits & 0b00000011) == 0) {
      RememberLine = -1;
      centerClearCnt = 0;
    } else if ((sensorBits & 0b00111100) != 0 && (sensorBits & 0b11000011) == 0) {
      if (++centerClearCnt >= CENTER_CLEAR_TICKS) {
        RememberLine = 0;
      }
    }
  }
}

void updateCalib() {
  for (int i = 0; i < 8; i++) {
    if (sensorValue[i] < black_value[i]) black_value[i] = sensorValue[i];
    if (sensorValue[i] > white_value[i]) white_value[i] = sensorValue[i];
    compare_value[i] = calcCompareThreshold(black_value[i], white_value[i]);
  }
}

void saveCalib() {
  Preferences p;
  if (!p.begin("calib", false)) return;
  p.putBytes("black", black_value, sizeof(black_value));
  p.putBytes("white", white_value, sizeof(white_value));
  p.putUChar("valid", 0xA5);
  p.end();
}

void readCalib() {
  Preferences p;
  bool valid = false;
  if (p.begin("calib", true)) {
    if (p.getUChar("valid", 0) == 0xA5) {
      size_t rb = p.getBytes("black", black_value, sizeof(black_value));
      size_t rw = p.getBytes("white", white_value, sizeof(white_value));
      if (rb == sizeof(black_value) && rw == sizeof(white_value)) {
        valid = true;
      }
    }
    p.end();
  }

  if (!valid) {
    for (int i = 0; i < 8; i++) {
      black_value[i]   = 200;
      white_value[i]   = 3500;
      compare_value[i] = calcCompareThreshold(black_value[i], white_value[i]);
    }
    saveCalib();
    return;
  }

  for (int i = 0; i < 8; i++) {
    if (white_value[i] <= black_value[i] + 50 || white_value[i] > 4095) {
      black_value[i] = 200;
      white_value[i] = 3500;
    }
    compare_value[i] = calcCompareThreshold(black_value[i], white_value[i]);
  }
}
