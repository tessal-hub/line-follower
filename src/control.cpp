#include "control.h"
#include "config.h"
#include "params.h"
#include "motor.h"
#include "sensor.h"
#include "encoders.h"

float realSpeedMmS = 0.0f;
float vEst = 0.0f;
float vModelPwm = 0.0f;
bool  encActive = false;
int   servoPwm = 0;
float dbgPTerm = 0.0f;
float dbgDTerm = 0.0f;
int   brakeHoldCnt = 0;
int   pivotBrakeTicks = 0;
int   currentRunState = STATE_IDLE;

static bool  isHighSpeedMode = false;
static float currentStraightSpeed = 0.0f;
static int   targetSpeedForward = 0;
static int   straightTicks = 0;
static bool  brakeArmed = false;
static int   lostLineTicks = 0;
static int   pivotTimeoutTicks = 0;
static uint32_t runStartMs = 0;

void controlInit() {
  currentRunState = STATE_IDLE;
  controlEmergencyStop();
}

void controlEmergencyStop() {
  motors_coast();           // Tự reset wheel-loop bên trong
  brakeHoldCnt = 0;
  pivotBrakeTicks = 0;
  brakeArmed = false;
  straightTicks = 0;
  lostLineTicks = 0;
  pivotTimeoutTicks = 0;
  currentStraightSpeed = 0.0f;
  targetSpeedForward = 0;
  currentRunState = STATE_IDLE;
}

void controlResetRun(bool isHighSpeed) {
  isHighSpeedMode = isHighSpeed;
  targetSpeedForward = isHighSpeed ? gParams.boostPwm : gParams.basePwm;
  currentStraightSpeed = (float)gParams.basePwm;
  vModelPwm = (float)gParams.cornerPwm;
  vEst = (float)gParams.cornerPwm;
  realSpeedMmS = 0.0f;
  encActive = false;
  brakeHoldCnt = 0;
  pivotBrakeTicks = 0;
  brakeArmed = false;
  straightTicks = 0;
  lostLineTicks = 0;
  pivotTimeoutTicks = 0;
  runStartMs = millis();
  currentRunState = STATE_RUN;

  resetSensorTracking();
  encodersResetTicks();
}

void controlCalculatePID(bool tick2ms) {
  // 1. Mô hình quán tính động cơ (cập nhật mỗi 2ms để giữ nguyên hằng số thời gian
  //    khi hàm được gọi ở tần số 2kHz)
  if (tick2ms) {
    float cmdF = (float)max(motorsGetCmdSpeed(), 0);
    float alphaModel = (cmdF >= vModelPwm) ? 0.025f : 0.012f;
    vModelPwm += (cmdF - vModelPwm) * alphaModel;

    // Hợp nhất vận tốc thực từ Encoder và mô hình
    if (encActive) {
      float encEquivPwm = max(0.0f, realSpeedMmS) * PWM_PER_MMS;
      vEst = 0.75f * encEquivPwm + 0.25f * vModelPwm;
    } else {
      vEst = vModelPwm;
    }
  }

  // Khi hoàn toàn không thấy line -> giữ nguyên servoPwm và đánh dấu wasLineLost
  if (sensorBits == 0b00000000) {
    dFiltered = 0.0f;
    wasLineLost = true;
    return;
  }

  // 2. Tính trọng tâm vị trí vạch đen [-10500 .. +10500]
  long sum = 0;
  long avg = 0;
  for (int j = 0; j < 8; j++) {
    avg += (long)sensorPID[j] * (j * 3000L);
    sum += (long)sensorPID[j];
  }

  long i = lastPos;
  if (sum > 400) {
    i = (avg / sum) - 10500;
  }

  // Chống bẫy trọng tâm khi cán vạch ngang góc vuông 90°
  if (cornerCrossbarDetected) {
    if (RememberLine == 1 && i < 7500)   i = 7500;
    if (RememberLine == -1 && i > -7500) i = -7500;
  }

  posPID = i;

  // Chống D-kick ở chu kỳ đầu tiên khi vừa bắt lại vạch
  if (wasLineLost) {
    lastPos = i;
    dFiltered = 0.0f;
    wasLineLost = false;
  }

  // Cập nhật đạo hàm mỗi 2ms kèm bộ lọc thông thấp EMA
  if (tick2ms) {
    float rawDelta = (float)(i - lastPos) * 4.0f;
    dFiltered = gTuning.dFilterAlpha * rawDelta + (1.0f - gTuning.dFilterAlpha) * dFiltered;
    lastPos = i;
  }

  // Deadband nhỏ quanh tâm giúp xe chạy thẳng đầm chắc
  long err = (abs(i) < gTuning.deadbandPos) ? 0 : i;

  // 3. Tự động Gain Scheduling theo tỷ số vận tốc r = vEst / basePwm
  float refSp = max((float)gParams.basePwm, 1200.0f);
  float r = constrain(vEst / refSp, 0.75f, 2.0f);

  float activeKp     = gParams.kp * (1.0f - 0.18f * (r - 1.0f));
  float activeKd     = gParams.kd * (1.0f + 0.75f * (r - 1.0f));
  float activeKCurve = gParams.kCurve * (1.0f + 0.30f * max(0.0f, r - 1.0f));
  if (activeKp < gParams.kp * 0.5f) activeKp = gParams.kp * 0.5f;

  // 4. Tính khâu P + D + K-Curve phi tuyến
  float pTerm = activeKp * (float)err;
  float dTerm = activeKd * dFiltered * gTuning.dTermScale;
  dbgPTerm = pTerm / gTuning.pidDivider;
  dbgDTerm = dTerm / gTuning.pidDivider;

  float curveTerm = 0.0f;
  long curveThreshold = constrain((long)gParams.brkTriggerPos - 200L, 1400L, 5000L);
  if (abs(err) > curveThreshold && activeKCurve > 0.0f) {
    float excess = (float)(abs(err) - curveThreshold);
    float boost = activeKCurve * (excess * excess) / 2800.0f;
    curveTerm = (err > 0) ? boost : -boost;
  }

  long iRet = (long)((pTerm + dTerm + curveTerm) / gTuning.pidDivider);
  servoPwm = constrain((int)iRet, -4095, 4095);
}

void controlUpdateRunforward(bool tick1ms, bool tick4ms) {
  // Lực xoay trục góc vuông 90° từ K-Curve
  float effKCurve = isHighSpeedMode ? (gParams.kCurve * gTuning.pivotKCurveScale) : gParams.kCurve;
  int extraTurn = (int)(effKCurve * gTuning.pivotTurnCoeff);
  int pivotFwd  = constrain(3000 + extraTurn, 2400, 4095);
  int pivotRev  = constrain(2200 + extraTurn, 1500, MAX_PLUG_PWM);

  // Vận tốc vật lý tham chiếu (mm/s)
  float pBrk            = constrain(gParams.brkPowerPct * 0.01f, 0.0f, 1.0f);
  float targetCornerPwm = (float)min(targetSpeedForward, gParams.cornerPwm);
  float cornerMmS       = targetCornerPwm / PWM_PER_MMS;
  float trigBrkMmS      = cornerMmS * (1.25f - 0.12f * pBrk);
  float trigPivotMmS    = cornerMmS * 1.05f;
  float exitBrkMmS      = cornerMmS * (0.95f - 0.10f * pBrk);
  float exitPivotMmS    = cornerMmS * 0.42f;

  long brkPosThr        = constrain((long)gParams.brkTriggerPos, 1500L, 6000L);
  long dashLimit        = constrain(brkPosThr + 1800L, 3500L, 7500L);
  int minStraightTicks  = constrain((int)roundf(14.0f - 8.0f * pBrk), 5, 16);

  // Kiểm tra vạch đích (Finish Line)
#if FINISH_LINE_ENABLE
  if (finishLineDetected && (millis() - runStartMs > FINISH_LINE_IGNORE_START_MS)) {
    motors_short_brake();
    currentRunState = STATE_FINISH_STOP;
    return;
  }
#endif

  switch (currentRunState) {
    case STATE_RUN: {
      if (sensorBits == 0b00000000) {
        // --- MẤT VẠCH ---
        if (RememberLine == 0 && abs(lastPos) < dashLimit) {
          // Vạch nét đứt: giữ hướng chạy qua khoảng trắng
          if (tick1ms) lostLineTicks++;
          if (lostLineTicks > 250) { // Quá 250ms mất line -> chuyển dừng an toàn
            currentRunState = STATE_SAFE_STOP;
            motors_coast();
          } else {
            int dashSpeed = min(targetSpeedForward, gParams.basePwm);
            handleAndSpeed(servoPwm / 4, dashSpeed);
          }
        } else {
          // Góc vuông 90° hoặc văng cua: chuẩn bị xoay trục (Pivot)
          motorsResetWheelLoop(); // Chuyển sang điều khiển PWM thô
          pivotTimeoutTicks = 0;
          straightTicks = 0;
          brakeArmed = false;
          currentStraightSpeed = targetCornerPwm;

          bool enteringHot = (vEst > targetCornerPwm * 1.10f) ||
                             (encActive && realSpeedMmS > trigPivotMmS);

          if (gParams.activeBrakeEn && enteringHot) {
            float gamma = encActive ? constrain(realSpeedMmS / max(cornerMmS, 100.0f), 1.0f, 2.2f)
                                    : constrain(vEst / max(targetCornerPwm, 600.0f), 1.0f, 2.2f);
            float baseBrkMs = 18.0f + 32.0f * pBrk;
            int totalBrkMs = constrain((int)roundf(baseBrkMs * (gamma * gamma - 0.80f)), 10, 95);
            pivotBrakeTicks = max(brakeHoldCnt, totalBrkMs);
          } else {
            pivotBrakeTicks = 0;
          }
          brakeHoldCnt = 0;

          if (RememberLine == 1 || (RememberLine == 0 && lastPos > 0)) {
            RememberLine = 1;
            currentRunState = STATE_PIVOT_RIGHT;
          } else {
            RememberLine = -1;
            currentRunState = STATE_PIVOT_LEFT;
          }
        }
      } else {
        // --- ĐANG CÓ LINE ---
        lostLineTicks = 0;
        long absPos = abs(posPID);
        float absD  = fabsf(dFiltered);

        // Phase-Lead prediction khi vạch trôi nhanh ra ngoài
        bool movingOutward = ((posPID > 0 && dFiltered > 0.0f) || (posPID < 0 && dFiltered < 0.0f));
        long predictedAbsPos = absPos;
        if (absPos >= 1800 && movingOutward) {
          predictedAbsPos += constrain((long)(absD * gTuning.leadFactor), 0L, gTuning.leadMaxOffset);
        }

        if (tick1ms) {
          if (predictedAbsPos >= brkPosThr || cornerCrossbarDetected) {
            bool fastEnoughToBrake = (straightTicks >= minStraightTicks) ||
                                     (encActive && realSpeedMmS > trigBrkMmS) ||
                                     (vEst > targetCornerPwm * 1.22f);

            if (gParams.activeBrakeEn && brakeArmed && brakeHoldCnt == 0 && fastEnoughToBrake) {
              motorsResetWheelLoop(); // Phanh ngược dùng PWM thô
              float gamma = encActive ? constrain(realSpeedMmS / max(cornerMmS, 100.0f), 1.0f, 2.2f)
                                      : constrain(vEst / max(targetCornerPwm, 600.0f), 1.0f, 2.2f);
              float baseBrkMs = 16.0f + 28.0f * pBrk;
              brakeHoldCnt = constrain((int)roundf(baseBrkMs * (gamma * gamma - 0.85f)), 8, 85);
              brakeArmed = false;
            }
            straightTicks = 0;
            currentStraightSpeed = min(currentStraightSpeed, targetCornerPwm);
          } else if (tick4ms && absPos < brkPosThr) {
            if (straightTicks < 500) straightTicks++;
            if (straightTicks >= minStraightTicks || (encActive && realSpeedMmS > trigBrkMmS)) {
              brakeArmed = true;
            }
            if (currentStraightSpeed < (float)targetSpeedForward) {
              currentStraightSpeed = min(currentStraightSpeed + gParams.accelStep, (float)targetSpeedForward);
            }
          }
        }

        // THỰC THI PHANH NGƯỢC BẤT ĐỐI XỨNG:
        if (brakeHoldCnt > 0) {
          if (tick1ms) brakeHoldCnt--;
          motorsSetCmdSpeed(0);

          // Thoát phanh dựa trực tiếp trên Encoder thực tế; mô hình chỉ dùng khi mất encoder
          bool speedDroppedOk = encActive ? (realSpeedMmS < exitBrkMmS)
                                          : (vEst < targetCornerPwm * 1.05f && brakeHoldCnt < 8);

          if (speedDroppedOk) {
            brakeHoldCnt = 0;
          } else {
            int turnDir = (posPID > 0) ? 1 : ((posPID < 0) ? -1 : RememberLine);
            float excessPwm = max(0.0f, vEst - targetCornerPwm);
            int basePlug  = constrain((int)roundf(1200.0f + 1400.0f * pBrk + 0.35f * excessPwm), 1200, MAX_PLUG_PWM);
            int steerPlug = (int)roundf(350.0f + 500.0f * pBrk);
            int innerPlug = -constrain(basePlug + steerPlug, 1500, MAX_PLUG_PWM);
            int outerPlug = -constrain(basePlug - (steerPlug / 3), 1000, MAX_PLUG_PWM - 200);

            if (turnDir > 0) {
              speed_run(outerPlug, innerPlug);
            } else if (turnDir < 0) {
              speed_run(innerPlug, outerPlug);
            } else {
              handleAndSpeed(constrain(servoPwm, -steerPlug, steerPlug), -basePlug);
            }
            break;
          }
        }

        // ĐIỀU TỐC TRONG CUA:
        int curSpeed = (int)currentStraightSpeed;
        if (absPos > 1800) {
          float span = max((float)(brkPosThr + 1200L - 1800L), 1500.0f);
          float k = constrain((float)(absPos - 1800) / span, 0.0f, 1.0f);
          curSpeed = (int)((float)curSpeed + (targetCornerPwm - (float)curSpeed) * k);
        }

        if (absPos > 1800 && movingOutward) {
          curSpeed -= (int)(absD * (gTuning.cornerDecelBase + gTuning.cornerDecelBrkScale * pBrk));
        }

        int minTorquePwm = max((int)(targetCornerPwm * 0.85f), gTuning.minTorqueFloor);
        curSpeed = constrain(curSpeed, minTorquePwm, 4095);
        handleAndSpeed(servoPwm, curSpeed);
      }
      break;
    }

    case STATE_PIVOT_RIGHT: {
      motorsSetCmdSpeed(0);
      if (tick1ms) pivotTimeoutTicks++;
      if (pivotTimeoutTicks > 300) {
        currentRunState = STATE_SAFE_STOP;
        motors_coast();
        break;
      }

      if (pivotBrakeTicks > 0) {
        if (tick1ms) pivotBrakeTicks--;
        bool canStartPivot = encActive ? (realSpeedMmS < exitPivotMmS)
                                       : (pivotBrakeTicks == 0);
        if (canStartPivot) {
          pivotBrakeTicks = 0;
          speed_run(pivotFwd, -pivotRev);
        } else {
          int plugHigh = constrain((int)roundf(2000.0f + 1000.0f * pBrk), 1800, MAX_PLUG_PWM);
          speed_run(-(plugHigh - 200), -plugHigh);
        }
      } else {
        speed_run(pivotFwd, -pivotRev);
      }

      if (sensorMask(0b00111100) != 0) {
        lastPos = 2200;
        wasLineLost = true;
        dFiltered = 0.0f;
        RememberLine = 0;
        straightTicks = 0;
        brakeHoldCnt = 0;
        pivotBrakeTicks = 0;
        brakeArmed = false;
        currentStraightSpeed = max(targetCornerPwm, 1500.0f);
        currentRunState = STATE_RUN;
        lostLineTicks = 0;
      }
      break;
    }

    case STATE_PIVOT_LEFT: {
      motorsSetCmdSpeed(0);
      if (tick1ms) pivotTimeoutTicks++;
      if (pivotTimeoutTicks > 300) {
        currentRunState = STATE_SAFE_STOP;
        motors_coast();
        break;
      }

      if (pivotBrakeTicks > 0) {
        if (tick1ms) pivotBrakeTicks--;
        bool canStartPivot = encActive ? (realSpeedMmS < exitPivotMmS)
                                       : (pivotBrakeTicks == 0);
        if (canStartPivot) {
          pivotBrakeTicks = 0;
          speed_run(-pivotRev, pivotFwd);
        } else {
          int plugHigh = constrain((int)roundf(2000.0f + 1000.0f * pBrk), 1800, MAX_PLUG_PWM);
          speed_run(-plugHigh, -(plugHigh - 200));
        }
      } else {
        speed_run(-pivotRev, pivotFwd);
      }

      if (sensorMask(0b00111100) != 0) {
        lastPos = -2200;
        wasLineLost = true;
        dFiltered = 0.0f;
        RememberLine = 0;
        straightTicks = 0;
        brakeHoldCnt = 0;
        pivotBrakeTicks = 0;
        brakeArmed = false;
        currentStraightSpeed = max(targetCornerPwm, 1500.0f);
        currentRunState = STATE_RUN;
        lostLineTicks = 0;
      }
      break;
    }

    case STATE_SAFE_STOP:
    case STATE_FINISH_STOP:
    default:
      motors_coast();
      break;
  }
}
