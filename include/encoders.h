#pragma once
#include <Arduino.h>

void encodersInit();
int32_t encodersGetLeftTicks();
int32_t encodersGetRightTicks();
void encodersResetTicks();

float encodersGetLeftDistanceMm();
float encodersGetRightDistanceMm();

void encodersUpdateSpeed();
float encodersGetLeftSpeedMmS();
float encodersGetRightSpeedMmS();
