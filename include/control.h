#pragma once
#include <Arduino.h>

enum RunState {
  STATE_IDLE        = 0,
  STATE_RUN         = 11,
  STATE_PIVOT_RIGHT = 21,
  STATE_PIVOT_LEFT  = 31,
  STATE_TEST_HOLD   = 91,
  STATE_SAFE_STOP   = 100,
  STATE_FINISH_STOP = 101
};

extern float realSpeedMmS;
extern float vEst;
extern float vModelPwm;
extern bool  encActive;
extern int   servoPwm;
extern float dbgPTerm;
extern float dbgDTerm;
extern int   brakeHoldCnt;
extern int   pivotBrakeTicks;
extern int   currentRunState;

void controlInit();
void controlResetRun(bool isHighSpeed);
void controlCalculatePID(bool tick2ms);
void controlUpdateRunforward(bool tick1ms, bool tick4ms);
void controlEmergencyStop();
