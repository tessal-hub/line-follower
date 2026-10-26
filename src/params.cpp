#include "params.h"
#include "motor.h"

// Tham số vận hành toàn cục
RobotParams gParams;

// Mảng 10 giá trị thô trên giao diện Menu OLED
float gMenuParams[P_COUNT] = {
  240.0f, // P_BOOST
  145.0f, // P_BASE
   95.0f, // P_CORNER
   15.0f, // P_KP
   60.0f, // P_KD
    6.5f, // P_KCURVE
   30.0f, // P_ACCEL
   80.0f, // P_BRK_PWR
 2600.0f, // P_BRK_POS
    1.0f  // P_WLOOP (1 = ON)
};

// Định nghĩa toàn bộ hằng số tuning
const TuningConstants gTuning = {
  .dTermScale          = 3.0f,
  .dFilterAlpha        = 0.18f,
  .pidDivider          = 40.0f,
  .deadbandPos         = 120L,
  .leadFactor          = 0.45f,
  .leadMaxOffset       = 1200L,
  .cornerDecelBase     = 0.20f,
  .cornerDecelBrkScale = 0.25f,
  .pivotKCurveScale    = 1.25f,
  .pivotTurnCoeff      = 45.0f,
  .minTorqueFloor      = 1350,
  .wlKp                = 0.40f,
  .wlKi                = 3.0f,
  .wlIntegMax          = 600.0f,
  .wlStaleUs           = 4000UL
};

void paramsSyncFromMenu() {
  int boostPwm  = (int)roundf(gMenuParams[P_BOOST]  * 16.0f);
  int basePwm   = (int)roundf(gMenuParams[P_BASE]   * 16.0f);
  int cornerPwm = (int)roundf(gMenuParams[P_CORNER] * 16.0f);

  // Ràng buộc giới hạn an toàn thực tế
  boostPwm  = constrain(boostPwm,  1280, 4095);
  basePwm   = constrain(basePwm,   1120, 4095);
  cornerPwm = constrain(cornerPwm, 1360, 4095);

  if (cornerPwm > basePwm) cornerPwm = basePwm;
  if (boostPwm < basePwm)   boostPwm = basePwm;

  gParams.boostPwm       = boostPwm;
  gParams.basePwm        = basePwm;
  gParams.cornerPwm      = cornerPwm;
  gParams.kp             = gMenuParams[P_KP];
  gParams.kd             = gMenuParams[P_KD];
  gParams.kCurve         = gMenuParams[P_KCURVE];
  gParams.accelStep      = gMenuParams[P_ACCEL];
  gParams.brkPowerPct    = gMenuParams[P_BRK_PWR];
  gParams.brkTriggerPos  = gMenuParams[P_BRK_POS];
  gParams.activeBrakeEn  = (gParams.brkPowerPct >= 5.0f);
  gParams.wheelLoopEn    = (gMenuParams[P_WLOOP] >= 0.5f);
  motorsSetWheelLoop(gParams.wheelLoopEn);
}

void paramsInit() {
  paramsSyncFromMenu();
}
