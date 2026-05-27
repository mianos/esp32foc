#pragma once

#include <atomic>

#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

// Phase-current sensing via INA240 shunt amplifiers on ADC1. Single instance;
// the motor commutation task drives sample_sync() at the PWM-synced moment.
class CurrentSense {
 public:
    struct Phases { float ia; float ib; float ic; };
    struct RawAdc { int a; int b; };

    esp_err_t init();

    // Call with the motor disabled to capture the zero-current bias point.
    esp_err_t calibrate_bias();

    // One fast ADC sample per phase, folded into the EMA filter. Designed to be
    // called from the motor commutation tick (PWM-synchronized, ~1 kHz) so
    // samples land at a consistent point in the carrier cycle where the inductor
    // current ripple crosses through the DC average. Safe before init() (no-op).
    void sample_sync();

    // Phase currents in amps from the EMA-filtered, PWM-synchronized samples.
    // Lock-free atomic reads — safe to call from any task at any rate. ic =
    // -(ia + ib).
    Phases read() const;

    // Single instantaneous ADC samples. Useful for verifying wiring; expect
    // rail-pinned values during PWM activity (use read() for the reliable
    // filtered values).
    RawAdc read_raw() const;

 private:
    int read_mv_fast(adc_channel_t ch) const;
    int read_mv_averaged(adc_channel_t ch, int n) const;

    adc_oneshot_unit_handle_t adc_ = nullptr;   // nullptr until init()
    int bias_a_mv_ = 1650;
    int bias_b_mv_ = 1650;
    std::atomic<float> filtered_a_amps_{0.0f};
    std::atomic<float> filtered_b_amps_{0.0f};
};
