#pragma once
#include <Arduino.h>

void motorsInit();
void speed_run(int speedDC_left, int speedDC_right);       // Ghi PWM thô (phanh/pivot)
void handleAndSpeed(int angle, int speed1);                // Lái + định tốc (có wheel closed-loop)
void motors_short_brake();
void motors_coast();

int  motorsGetCmdSpeed();
void motorsSetCmdSpeed(int speed);

// Vòng lặp kín vận tốc bánh xe (FF + PI, đo từ Encoder)
void motorsSetWheelLoop(bool enabled);
void motorsResetWheelLoop();
