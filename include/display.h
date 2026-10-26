#pragma once
#include <Arduino.h>
#include "params.h"

void displayInit();
void displayMenuLoop();
bool displayIsMenuActive();
bool displayTestHoldRequested();
void displaySetCalibProgress(uint32_t elapsedMs, uint32_t totalMs);
void displayShowMessage(const char *title, const char *msg, uint32_t durationMs = 1500);
void displayShowStatus(const char *mode, int speed, int pos, unsigned char sensorBits);
void displayShowCalibLive(const unsigned int raw[8], const unsigned int minVal[8], const unsigned int maxVal[8]);
