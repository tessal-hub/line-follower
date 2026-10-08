#pragma once
#include <Arduino.h>

void motorsInit();
void motorsSet(int leftPwm, int rightPwm);
void motorsStop();
void motorsBrake();
void handleAndSpeed(int steer, int baseSpeed, float absErr, int &outLeft, int &outRight);
