#pragma once
#include <Arduino.h>
enum RobotState : uint8_t {
    STATE_RUN = 0,
    STATE_SHARP_TURN = 1,
    STATE_RECOVERY = 2
};

struct StateMachineContext {
    RobotState currentState;
    float lastValidErrorMm;
    int8_t lastValidDir;      // +1 = line lệch phải, -1 = line lệch trái
    float lastValidSpeedMmS;
    uint32_t stateEnterTimeMs;
    uint32_t recoveryStartTimeMs;
    bool emergencyStopTriggered;
};

void stateMachineInit();
void stateMachineReset();

// Cập nhật máy trạng thái 3 State (RUN, SHARP_TURN, RECOVERY) kèm hysteresis
StateMachineContext stateMachineUpdate(
    bool lineDetected,
    float errorMm,
    float dErrorMmS,
    float confidence,
    bool isExtrapolated,
    int turn90,
    float currentSpeedMmS,
    uint32_t nowMs
);

RobotState stateMachineGetCurrentState();
