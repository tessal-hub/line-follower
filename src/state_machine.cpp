#include "state_machine.h"
#include "config.h"

static StateMachineContext ctx;
static const uint32_t T_RECOVERY_MAX_MS = LINE_LOST_TIMEOUT_MS; // 700ms

// Ngưỡng Hysteresis cho SHARP_TURN:
static const float SHARP_ENTER_E_MM = 12.0f; // Bắt đầu vào cua gắt sớm từ 12mm (mắt ch3/ch4 chớm lệch)
static const float SHARP_EXIT_E_MM  = 6.5f;  // Thoát cua gắt về bám thẳng khi đã về tâm 6.5mm

// Bộ lọc chống nhiễu chuyển trạng thái
static uint8_t lostLineCycles = 0;

void stateMachineInit() {
    stateMachineReset();
}

void stateMachineReset() {
    ctx.currentState = STATE_RUN;
    ctx.lastValidErrorMm = 0.0f;
    ctx.lastValidDir = 1;
    ctx.lastValidSpeedMmS = 900.0f;
    ctx.stateEnterTimeMs = millis();
    ctx.recoveryStartTimeMs = 0;
    ctx.emergencyStopTriggered = false;
    lostLineCycles = 0;
}

StateMachineContext stateMachineUpdate(
    bool lineDetected,
    float errorMm,
    float dErrorMmS,
    float confidence,
    bool isExtrapolated,
    int turn90,
    float currentSpeedMmS,
    uint32_t nowMs
) {
    // 1. Cập nhật thông tin hợp lệ gần nhất khi còn thấy vạch
    if (lineDetected) {
        ctx.lastValidErrorMm = errorMm;
        ctx.lastValidSpeedMmS = currentSpeedMmS;
        if (errorMm > 5.0f) {
            ctx.lastValidDir = 1;
        } else if (errorMm < -5.0f) {
            ctx.lastValidDir = -1;
        }
        lostLineCycles = 0;
    } else {
        lostLineCycles++;
    }

    // 2. Chuyển trạng thái với Hysteresis & Debounce 8 chu kỳ (32ms)
    static const uint8_t LOST_LINE_CONFIRM_CYCLES = 8; // 32ms chống chớp chập chờn

    switch (ctx.currentState) {
        case STATE_RUN: {
            if (lostLineCycles >= LOST_LINE_CONFIRM_CYCLES) {
                // Chỉ vào RECOVERY khi thực sự mất vạch liên tục > 32ms
                ctx.currentState = STATE_RECOVERY;
                ctx.recoveryStartTimeMs = nowMs;
                ctx.stateEnterTimeMs = nowMs;
            } else if (fabsf(errorMm) > SHARP_ENTER_E_MM || isExtrapolated || turn90 != 0) {
                // Vào SHARP_TURN khi vạch sắp tràn mép hoặc gặp cua vuông
                ctx.currentState = STATE_SHARP_TURN;
                ctx.stateEnterTimeMs = nowMs;
            }
            break;
        }

        case STATE_SHARP_TURN: {
            if (lostLineCycles >= LOST_LINE_CONFIRM_CYCLES) {
                ctx.currentState = STATE_RECOVERY;
                ctx.recoveryStartTimeMs = nowMs;
                ctx.stateEnterTimeMs = nowMs;
            } else if (fabsf(errorMm) < SHARP_EXIT_E_MM && !isExtrapolated && turn90 == 0 && fabsf(dErrorMmS) < 250.0f) {
                // Thoát SHARP_TURN về RUN khi vạch đã quay về tâm an toàn (Hysteresis EXIT < ENTER)
                ctx.currentState = STATE_RUN;
                ctx.stateEnterTimeMs = nowMs;
            }
            break;
        }

        case STATE_RECOVERY: {
            // Kiểm tra timeout mất vạch: Dừng xe an toàn sau T_RECOVERY_MAX_MS (700ms)
            if (nowMs - ctx.recoveryStartTimeMs > T_RECOVERY_MAX_MS) {
                ctx.emergencyStopTriggered = true;
            } else if (lineDetected) {
                // ĐÃ BẮT LẠI ĐƯỢC LINE! Thoát ngay lập tức khỏi RECOVERY
                ctx.recoveryStartTimeMs = 0;
                ctx.stateEnterTimeMs = nowMs;
                lostLineCycles = 0;
                if (fabsf(errorMm) > SHARP_EXIT_E_MM || isExtrapolated || turn90 != 0) {
                    ctx.currentState = STATE_SHARP_TURN;
                } else {
                    ctx.currentState = STATE_RUN;
                }
            }
            break;
        }
    }

    return ctx;
}

RobotState stateMachineGetCurrentState() {
    return ctx.currentState;
}
