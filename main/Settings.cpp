#include "Settings.h"

#include <cstdlib>

namespace {

bool parse_float(const std::string &s, float &out) {
    char *end = nullptr;
    float v = std::strtof(s.c_str(), &end);
    if (end == s.c_str()) return false;
    out = v;
    return true;
}

bool parse_int(const std::string &s, int &out) {
    char *end = nullptr;
    long v = std::strtol(s.c_str(), &end, 10);
    if (end == s.c_str()) return false;
    out = static_cast<int>(v);
    return true;
}

}  // namespace

bool Settings::load_cal(Motor::Cal &cal) const {
    std::string rs_s, ls_s, vd_s;
    if (!retrieve("rs_ohm",   rs_s) || rs_s.empty()) return false;
    if (!retrieve("ls_henry", ls_s) || ls_s.empty()) return false;
    if (!retrieve("v_dead",   vd_s) || vd_s.empty()) return false;
    if (!parse_float(rs_s, cal.rs_ohm))   return false;
    if (!parse_float(ls_s, cal.ls_henry)) return false;
    if (!parse_float(vd_s, cal.v_dead))   return false;
    cal.valid = true;
    return true;
}

bool Settings::save_cal(const Motor::Cal &cal) {
    bool ok = true;
    ok &= store("rs_ohm",   std::to_string(cal.rs_ohm));
    ok &= store("ls_henry", std::to_string(cal.ls_henry));
    ok &= store("v_dead",   std::to_string(cal.v_dead));
    return ok;
}

void Settings::load_vhz(float &v_offset, float &v_per_rad_s, float &stall_a) const {
    v_offset    = kDefaultVOffsetV;
    v_per_rad_s = kDefaultVPerRadS;
    stall_a     = Motor::kDefaultStallCurrentA;
    std::string s;
    if (retrieve("v_offset",   s) && !s.empty()) parse_float(s, v_offset);
    if (retrieve("v_per_rads", s) && !s.empty()) parse_float(s, v_per_rad_s);
    if (retrieve("stall_a",    s) && !s.empty()) parse_float(s, stall_a);
}

bool Settings::save_vhz(float v_offset, float v_per_rad_s, float stall_a) {
    bool ok = true;
    ok &= store("v_offset",   std::to_string(v_offset));
    ok &= store("v_per_rads", std::to_string(v_per_rad_s));
    ok &= store("stall_a",    std::to_string(stall_a));
    return ok;
}

void Settings::load_pump_range(float &min_rad_s, float &max_rad_s) const {
    min_rad_s = kDefaultPumpMinRadS;
    max_rad_s = kDefaultPumpMaxRadS;
    std::string s;
    if (retrieve("pump_min", s) && !s.empty()) parse_float(s, min_rad_s);
    if (retrieve("pump_max", s) && !s.empty()) parse_float(s, max_rad_s);
}

bool Settings::save_pump_range(float min_rad_s, float max_rad_s) {
    bool ok = true;
    ok &= store("pump_min", std::to_string(min_rad_s));
    ok &= store("pump_max", std::to_string(max_rad_s));
    return ok;
}

int Settings::load_pole_pairs() const {
    int pole_pairs = kDefaultPolePairs;
    std::string s;
    if (retrieve("pole_pairs", s) && !s.empty()) parse_int(s, pole_pairs);
    return pole_pairs;
}

bool Settings::save_pole_pairs(int pole_pairs) {
    return store("pole_pairs", std::to_string(pole_pairs));
}

std::string Settings::load_hostname(const std::string &def) const {
    std::string hn;
    if (retrieve("hostname", hn) && !hn.empty()) return hn;
    return def;
}

bool Settings::save_hostname(const std::string &hostname) {
    return store("hostname", hostname);
}
