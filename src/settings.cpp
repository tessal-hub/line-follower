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
    if (!carPrefs.begin("car_cfg", false)) { // Open Read-Write to allow automatic migration
        Serial.println("[CONFIG] NVS 'car_cfg' namespace open failed. Using hardcoded defaults.");
        return;
    }

    bool isBrandNew = !carPrefs.isKey("init_v4") && !carPrefs.isKey("init_v3") && !carPrefs.isKey("init");

    if (isBrandNew) {
        Serial.println("[CONFIG] Fresh NVS Flash detected. Saving initial v4 defaults...");
        carPrefs.putFloat("bst_spd", cfg.boostSpeedMmS);
        carPrefs.putFloat("base_spd", cfg.baseSpeedMmS);
        carPrefs.putFloat("cnr_spd",  cfg.cornerSpeedMmS);
        carPrefs.putFloat("kp_gain",  cfg.kpGain);
        carPrefs.putFloat("kd_gain",  cfg.kdGain);
        carPrefs.putFloat("kc_gain",  cfg.kCurveGain);
        carPrefs.putBool("act_brk",   cfg.activeBrakeEn);
        carPrefs.putUChar("init_v4", 1);
        carPrefs.end();
        return;
    }

    // Preserve existing user tuning for speeds and ABS if already saved
    cfg.boostSpeedMmS  = carPrefs.getFloat("bst_spd", TARGET_BOOST_SPEED_MMS);
    cfg.baseSpeedMmS   = carPrefs.getFloat("base_spd", TARGET_BASE_SPEED_MMS);
    cfg.cornerSpeedMmS = carPrefs.getFloat("cnr_spd",  TARGET_CORNER_SPEED_MMS);
    cfg.activeBrakeEn  = carPrefs.getBool("act_brk",   true);

    if (!carPrefs.isKey("init_v4")) {
        // Migration to v4: Upgrade old geometric gains (Kp ~ 20 -> 45, Kd -> 92)
        Serial.println("[CONFIG] Migrating to v4: Updating steering gains (Kp=45, Kd=92) while preserving user speeds...");
        cfg.kpGain     = KP_GAIN;
        cfg.kdGain     = KD_GAIN;
        cfg.kCurveGain = K_CURVE_GAIN;
        carPrefs.putFloat("kp_gain", cfg.kpGain);
        carPrefs.putFloat("kd_gain", cfg.kdGain);
        carPrefs.putFloat("kc_gain", cfg.kCurveGain);
        carPrefs.putUChar("init_v4", 1);
    } else {
        // v4 already initialized: load user's custom gains from NVS
        cfg.kpGain     = carPrefs.getFloat("kp_gain",  KP_GAIN);
        cfg.kdGain     = carPrefs.getFloat("kd_gain",  KD_GAIN);
        cfg.kCurveGain = carPrefs.getFloat("kc_gain",  K_CURVE_GAIN);
    }
    carPrefs.end();

    // Unified bounds clamp
    cfg.baseSpeedMmS   = constrain(cfg.baseSpeedMmS,   CFG_BASE_SPD_MIN,   CFG_BASE_SPD_MAX);
    cfg.cornerSpeedMmS = constrain(cfg.cornerSpeedMmS, CFG_CORNER_SPD_MIN, CFG_CORNER_SPD_MAX);
    cfg.boostSpeedMmS  = constrain(cfg.boostSpeedMmS,  cfg.baseSpeedMmS,   CFG_BOOST_SPD_MAX);
    cfg.kpGain         = constrain(cfg.kpGain,         CFG_KP_MIN,         CFG_KP_MAX);
    cfg.kdGain         = constrain(cfg.kdGain,         CFG_KD_MIN,         CFG_KD_MAX);
    cfg.kCurveGain     = constrain(cfg.kCurveGain,     CFG_KC_MIN,         CFG_KC_MAX);

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
    carPrefs.putUChar("init_v4", 1);
    carPrefs.end();

    Serial.printf("[CONFIG] Saved to Flash NVS -> Boost:%.0f Base:%.0f Cnr:%.0f Kp:%.1f Kd:%.1f Kc:%.1f ABS:%s\n",
        cfg.boostSpeedMmS, cfg.baseSpeedMmS, cfg.cornerSpeedMmS, cfg.kpGain, cfg.kdGain, cfg.kCurveGain,
        cfg.activeBrakeEn ? "ON" : "OFF");
}
