#pragma once
#include <Arduino.h>

extern unsigned int sensorValue[8];
extern unsigned int sensorPID[8];
extern unsigned int black_value[8];
extern unsigned int white_value[8];
extern unsigned int compare_value[8];

extern unsigned char sensorBits;
extern long posPID;
extern long lastPos;
extern int  RememberLine;
extern bool wasLineLost;
extern float dFiltered;
extern bool cornerCrossbarDetected;
extern bool finishLineDetected;

void sensorsInit();
void read_sensor();
void updateCalib();
void saveCalib();
void readCalib();
void resetSensorTracking();

unsigned char sensorMask(unsigned char mask);
int getSensorActiveCount();
