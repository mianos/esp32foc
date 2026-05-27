#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t motor_init(void);
void motor_enable(void);
void motor_disable(void);

// Open-loop V/Hz drive: vamp = v_offset + v_per_rad_s * |current_velocity|.
// At standstill (ω = 0) vamp = v_offset, so set v_offset just above V_dead
// to keep startup current bounded. v_per_rad_s should track the motor's
// back-EMF constant Ke.
void motor_set_velocity(float rad_per_sec);
void motor_set_voltage_amplitude(float volts);   // sets v_offset
void motor_set_v_per_rad_s(float v_per_rad_s);
void motor_set_stall_current_a(float amps);      // trip threshold for stall detector

typedef struct {
    float target_velocity_rad_s;
    float current_velocity_rad_s;
    float voltage_amplitude_v;
    float v_offset_v;
    float v_per_rad_s;
    float i_mag_a;          // raw current vector magnitude, slow-averaged
    float i_bus_est_a;      // estimated bus current = (3/2)*vamp*|I|/Vbus
    float stall_current_a;  // currently-configured trip threshold
    bool  enabled;
    bool  stalled;          // sticky flag set when stall detection fires
    uint32_t uptime_s;
} motor_status_t;

void motor_get_status(motor_status_t *out);

typedef struct {
    bool  valid;
    float rs_ohm;
    float ls_henry;
    float v_dead;
} motor_cal_t;

// Run identification with the motor stationary. Measures Rs, Ls, V_dead by
// applying small DC voltages and a step input across the B-C winding pair.
// Diagnostic / informational: the cal doesn't directly drive control, but
// the V_dead estimate is used to size the rotor-align voltage at enable.
esp_err_t motor_identify(motor_cal_t *out);
void motor_set_cal(const motor_cal_t *cal);
void motor_get_cal(motor_cal_t *out);

#ifdef __cplusplus
}
#endif
