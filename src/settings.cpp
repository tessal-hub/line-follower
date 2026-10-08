#include "config.h"
#include <Preferences.h>

static Preferences carPrefs;

void configResetDefaults(CarConfig &cfg) {
    cfg.boostSpeedMmS  = TARGET_BOOST_SPEED_MMS;
    cfg.baseSpeedMmS   = TARGET_BASE_SPEED_MMS;
    cfg.cornerSpeedMmS = TARGET_CORNER_SPEED_MMS;
    cfg.kpGain         = KP_GAIN;
    cfg.kdGain         = KD_GAIN;
    cfg.kCurveGain     = K_CURVE_GAIN;
    cfg.activeBrakeEn  = true;
}

void configLoad(CarConfig &cfg) {
    configResetDefaults(cfg);
    if (!carPrefs.begin("car_cfg", true)) { // Read-only mode
        Serial.println("[CONFIG] NVS 'car_cfg' namespace open failed. Using hardcoded defaults.");
        return;
    }

    if (!carPrefs.isKey("init")) {
        Serial.println("[CONFIG] No saved config found in NVS Flash. Writing initial defaults...");
        carPrefs.end();
        configSave(cfg);
        return;
    }

    cfg.boostSpeedMmS  = carPrefs.getFloat("bst_spd", TARGET_BOOST_SPEED_MMS);
    cfg.baseSpeedMmS   = carPrefs.getFloat("base_spd", TARGET_BASE_SPEED_MMS);
    cfg.cornerSpeedMmS = carPrefs.getFloat("cnr_spd",  TARGET_CORNER_SPEED_MMS);
    cfg.kpGain         = carPrefs.getFloat("kp_gain",  KP_GAIN);
    cfg.kdGain         = carPrefs.getFloat("kd_gain",  KD_GAIN);
    cfg.kCurveGain     = carPrefs.getFloat("kc_gain",  K_CURVE_GAIN);
    cfg.activeBrakeEn  = carPrefs.getBool("act_brk",   true);
    carPrefs.end();

    // Open bounds: allows unrestricted tuning (floor limits removed)
    cfg.boostSpeedMmS  = constrain(cfg.boostSpeedMmS,  0.0f,   2500.0f);
    cfg.baseSpeedMmS   = constrain(cfg.baseSpeedMmS,   100.0f, 2200.0f);
    cfg.cornerSpeedMmS = constrain(cfg.cornerSpeedMmS, 100.0f, 1500.0f);
    cfg.kpGain         = constrain(cfg.kpGain,         0.0f,   80.0f);
    cfg.kdGain         = constrain(cfg.kdGain,         0.0f,   80.0f);
    cfg.kCurveGain     = constrain(cfg.kCurveGain,     0.0f,   25.0f);

    Serial.printf("[CONFIG] Loaded from Flash NVS -> Boost:%.0f Base:%.0f Cnr:%.0f Kp:%.1f Kd:%.1f Kc:%.1f ABS:%s\n",
        cfg.boostSpeedMmS, cfg.baseSpeedMmS, cfg.cornerSpeedMmS, cfg.kpGain, cfg.kdGain, cfg.kCurveGain,
        cfg.activeBrakeEn ? "ON" : "OFF");
}

void configSave(const CarConfig &cfg) {
    if (!carPrefs.begin("car_cfg", false)) { // Read-write mode
        Serial.println("[CONFIG] ERROR: Failed to open NVS 'car_cfg' for writing!");
        return;
    }

    carPrefs.putFloat("bst_spd", cfg.boostSpeedMmS);
    carPrefs.putFloat("base_spd", cfg.baseSpeedMmS);
    carPrefs.putFloat("cnr_spd",  cfg.cornerSpeedMmS);
    carPrefs.putFloat("kp_gain",  cfg.kpGain);
    carPrefs.putFloat("kd_gain",  cfg.kdGain);
    carPrefs.putFloat("kc_gain",  cfg.kCurveGain);
    carPrefs.putBool("act_brk",   cfg.activeBrakeEn);
    carPrefs.putUChar("init", 1);
    carPrefs.end();

    Serial.printf("[CONFIG] Saved to Flash NVS -> Boost:%.0f Base:%.0f Cnr:%.0f Kp:%.1f Kd:%.1f Kc:%.1f ABS:%s\n",
        cfg.boostSpeedMmS, cfg.baseSpeedMmS, cfg.cornerSpeedMmS, cfg.kpGain, cfg.kdGain, cfg.kCurveGain,
        cfg.activeBrakeEn ? "ON" : "OFF");
}
