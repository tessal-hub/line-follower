#include "config.h"
#include <Preferences.h>

static Preferences carPrefs;

void configResetDefaults(CarConfig &cfg) {
    cfg.boostSpeed  = DEFAULT_BOOST_SPEED;
    cfg.baseSpeed   = DEFAULT_BASE_SPEED;
    cfg.cornerSpeed = DEFAULT_CORNER_SPEED;
    cfg.kpGain      = KP_GAIN;
    cfg.kdGain      = KD_GAIN;
    cfg.kCurveGain  = K_CURVE_GAIN;
    cfg.activeBrakeEn = true;
}

void configLoad(CarConfig &cfg) {
    configResetDefaults(cfg);
    if (!carPrefs.begin("car_cfg", false)) {
        Serial.println("[CONFIG] NVS 'car_cfg' open failed. Using defaults.");
        return;
    }

    bool hasConfig = carPrefs.isKey("init_pd_v2");
    if (!hasConfig) {
        Serial.println("[CONFIG] Fresh NVS Flash. Saving initial PD defaults...");
        carPrefs.putFloat("bst_spd", cfg.boostSpeed);
        carPrefs.putFloat("base_spd", cfg.baseSpeed);
        carPrefs.putFloat("cnr_spd",  cfg.cornerSpeed);
        carPrefs.putFloat("kp_gain",  cfg.kpGain);
        carPrefs.putFloat("kd_gain",  cfg.kdGain);
        carPrefs.putFloat("kc_gain",  cfg.kCurveGain);
        carPrefs.putBool("act_brk",   cfg.activeBrakeEn);
        carPrefs.putUChar("init_pd_v2", 1);
        carPrefs.end();
        return;
    }

    cfg.boostSpeed  = carPrefs.getFloat("bst_spd", DEFAULT_BOOST_SPEED);
    cfg.baseSpeed   = carPrefs.getFloat("base_spd", DEFAULT_BASE_SPEED);
    cfg.cornerSpeed = carPrefs.getFloat("cnr_spd",  DEFAULT_CORNER_SPEED);
    cfg.kpGain      = carPrefs.getFloat("kp_gain",  KP_GAIN);
    cfg.kdGain      = carPrefs.getFloat("kd_gain",  KD_GAIN);
    cfg.kCurveGain  = carPrefs.getFloat("kc_gain",  K_CURVE_GAIN);
    cfg.activeBrakeEn = carPrefs.getBool("act_brk", true);
    carPrefs.end();

    // Bounds clamp
    cfg.baseSpeed   = constrain(cfg.baseSpeed,   CFG_BASE_SPD_MIN,   CFG_BASE_SPD_MAX);
    cfg.cornerSpeed = constrain(cfg.cornerSpeed, CFG_CORNER_SPD_MIN, CFG_CORNER_SPD_MAX);
    cfg.boostSpeed  = constrain(cfg.boostSpeed,  cfg.baseSpeed,      CFG_BOOST_SPD_MAX);
    cfg.kpGain      = constrain(cfg.kpGain,      CFG_KP_MIN,         CFG_KP_MAX);
    cfg.kdGain      = constrain(cfg.kdGain,      CFG_KD_MIN,         CFG_KD_MAX);
    cfg.kCurveGain  = constrain(cfg.kCurveGain,  CFG_KC_MIN,         CFG_KC_MAX);

    Serial.printf("[CONFIG] NVS Loaded -> Boost:%.0f Base:%.0f Cnr:%.0f Kp:%.1f Kd:%.1f Kc:%.1f ABS:%s\n",
        cfg.boostSpeed, cfg.baseSpeed, cfg.cornerSpeed, cfg.kpGain, cfg.kdGain, cfg.kCurveGain,
        cfg.activeBrakeEn ? "ON" : "OFF");
}

void configSave(const CarConfig &cfg) {
    if (!carPrefs.begin("car_cfg", false)) {
        Serial.println("[CONFIG] ERROR: Failed to open NVS 'car_cfg' for writing!");
        return;
    }

    carPrefs.putFloat("bst_spd", cfg.boostSpeed);
    carPrefs.putFloat("base_spd", cfg.baseSpeed);
    carPrefs.putFloat("cnr_spd",  cfg.cornerSpeed);
    carPrefs.putFloat("kp_gain",  cfg.kpGain);
    carPrefs.putFloat("kd_gain",  cfg.kdGain);
    carPrefs.putFloat("kc_gain",  cfg.kCurveGain);
    carPrefs.putBool("act_brk",   cfg.activeBrakeEn);
    carPrefs.putUChar("init_pd_v2", 1);
    carPrefs.end();

    Serial.printf("[CONFIG] NVS Saved -> Boost:%.0f Base:%.0f Cnr:%.0f Kp:%.1f Kd:%.1f Kc:%.1f ABS:%s\n",
        cfg.boostSpeed, cfg.baseSpeed, cfg.cornerSpeed, cfg.kpGain, cfg.kdGain, cfg.kCurveGain,
        cfg.activeBrakeEn ? "ON" : "OFF");
}
