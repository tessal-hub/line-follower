#pragma once
#include <Arduino.h>

void motorsInit();
void motorsSet(int leftPwm, int rightPwm);
void motorsStop();
void motorsBrake();
void motorsSetActiveBrake(bool enable);
