#pragma once

#include <string>

#include "NvsStorageManager.h"
#include "motor.h"   // Motor::Cal, Motor::kDefaultStallCurrentA

// robofoc-local persisted settings, layered on the shared NvsStorageManager
// (a string key/value store) with the typed accessors, NVS keys, and defaults
// all in one place. Derives from NvsStorageManager so the common component
// stays generic — app-specific keys live here, not in the shared class.
class Settings : public NvsStorageManager {
public:
    explicit Settings(const std::string &ns = "storage") : NvsStorageManager(ns) {}

    // Motor identification cal (Rs / Ls / V_dead). Returns false if any key
    // is missing or unparseable, leaving `cal` untouched of validity.
    bool load_cal(Motor::Cal &cal) const;
    bool save_cal(const Motor::Cal &cal);

    // V/Hz tuning + stall threshold. Missing keys fall back to defaults.
    void load_vhz(float &v_offset, float &v_per_rad_s, float &stall_a) const;
    bool save_vhz(float v_offset, float v_per_rad_s, float stall_a);

    // Pump duty->velocity mapping range. Missing keys fall back to defaults.
    void load_pump_range(float &min_rad_s, float &max_rad_s) const;
    bool save_pump_range(float min_rad_s, float max_rad_s);

    // Pole-pair count (RPM display only). Returns the default if unset.
    int  load_pole_pairs() const;
    bool save_pole_pairs(int pole_pairs);

    // Device hostname. Returns `def` if unset/empty.
    std::string load_hostname(const std::string &def) const;
    bool save_hostname(const std::string &hostname);

    static constexpr float kDefaultVOffsetV    = 0.10f;
    static constexpr float kDefaultVPerRadS    = 0.001f;
    static constexpr float kDefaultPumpMinRadS = 50.0f;
    static constexpr float kDefaultPumpMaxRadS = 1000.0f;
    static constexpr int   kDefaultPolePairs   = 7;
};
