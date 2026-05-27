#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t current_sense_init(void);

// Call with the motor disabled to capture the zero-current bias point.
esp_err_t current_sense_calibrate_bias(void);

// Take one fast ADC sample per phase, update internal EMA-filtered state.
// Designed to be called from the motor commutation tick (PWM-synchronized,
// ~1 kHz) so samples land at a consistent point in the carrier cycle where
// the inductor current ripple crosses through the DC average. Safe to call
// before current_sense_init() (no-op).
void current_sense_sample_sync(void);

// Phase currents in amps from the EMA-filtered, PWM-synchronized samples.
// Lock-free atomic reads - safe to call from any task at any rate. Returns
// the most recent filtered values. ic = -(ia + ib).
void current_sense_read(float *ia, float *ib, float *ic);

// Single instantaneous ADC samples. Useful for verifying wiring; expect
// rail-pinned values during PWM activity (use current_sense_read() for the
// reliable filtered values).
void current_sense_read_raw(int *raw_a, int *raw_b);

#ifdef __cplusplus
}
#endif
