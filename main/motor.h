#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <optional>

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class CurrentSense;

// Open-loop V/Hz BLDC drive with rotor-align-on-enable and stall detection.
// Single instance, constructed with a reference to the shared CurrentSense it
// samples once per commutation tick.
class Motor {
 public:
    static constexpr float    kDefaultStallCurrentA = 3.0f;

    struct Status {
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
    };

    struct Cal {
        bool  valid    = false;
        float rs_ohm   = 0.0f;
        float ls_henry = 0.0f;
        float v_dead   = 0.0f;
    };

    explicit Motor(CurrentSense &current_sense);

    esp_err_t init();
    void enable();
    void disable();

    // Open-loop V/Hz drive: vamp = v_offset + v_per_rad_s * |current_velocity|.
    // At standstill (ω = 0) vamp = v_offset, so set v_offset just above V_dead
    // to keep startup current bounded. v_per_rad_s should track the motor's
    // back-EMF constant Ke.
    void set_velocity(float rad_per_sec);
    void set_voltage_amplitude(float volts);   // sets v_offset
    void set_v_per_rad_s(float v_per_rad_s);
    void set_stall_current_a(float amps);      // trip threshold for stall detector

    Status status() const;

    // Run identification with the motor stationary. Measures Rs, Ls, V_dead by
    // applying small DC voltages and a step input across the B-C winding pair.
    // Diagnostic / informational: the cal doesn't directly drive control, but
    // the V_dead estimate is used to size the rotor-align voltage at enable.
    // Returns nullopt on failure (motor enabled, no current, bad fit — see log).
    std::optional<Cal> identify();
    void set_cal(const Cal &cal);
    Cal  cal() const;

 private:
    static constexpr int      kSinLutSize        = 256;
    static constexpr uint32_t kDecimationDefault = 20;

    void  setup_mcpwm();
    void  run();
    void  write_duties(float da, float db, float dc);
    void  ident_apply_voltage_bc(float vbc);
    float fast_sin(float angle_rad) const;

    static void task_main(void *arg);
    static void mcpwm_init_main(void *arg);
    static bool on_pwm_peak(mcpwm_timer_handle_t t,
                            const mcpwm_timer_event_data_t *e,
                            void *user_ctx);

    CurrentSense &current_sense_;

    std::array<float, kSinLutSize> sin_lut_{};

    mcpwm_timer_handle_t               timer_ = nullptr;
    std::array<mcpwm_oper_handle_t, 3> oper_{};
    std::array<mcpwm_cmpr_handle_t, 3> cmp_{};
    std::array<mcpwm_gen_handle_t, 3>  gen_{};

    SemaphoreHandle_t tick_sem_        = nullptr;
    SemaphoreHandle_t mcpwm_init_done_ = nullptr;

    // Shared state. Float / int writes to 4-byte-aligned addresses are atomic
    // on Xtensa, so plain load / store is enough.
    std::atomic<float>    target_velocity_rad_s_{0.0f};
    std::atomic<float>    current_velocity_rad_s_{0.0f};
    std::atomic<float>    voltage_amplitude_v_{0.0f};
    std::atomic<float>    v_offset_v_{0.0f};
    std::atomic<float>    v_per_rad_s_{0.0f};
    std::atomic<float>    stall_current_a_{kDefaultStallCurrentA};
    std::atomic<float>    i_mag_filtered_a_{0.0f};
    std::atomic<float>    i_bus_est_a_{0.0f};
    std::atomic<bool>     enabled_{false};
    std::atomic<bool>     stalled_{false};
    std::atomic<uint32_t> uptime_s_{0};

    // identify() takes ownership of the bridge briefly. The motor task observes
    // this flag and skips its loop body while it's set.
    std::atomic<bool>     ident_in_progress_{false};

    // 20 kHz carrier divided down to the commutation tick rate. Runtime
    // variable so identify() can drop to 1 (full 20 kHz sampling) during its
    // L-step capture. Read in the carrier-peak ISR.
    std::atomic<uint32_t> commutation_decimation_{kDecimationDefault};
    uint32_t isr_decim_ = 0;   // ISR-private divide-down counter

    // Loaded from /calibrate or NVS. Used to size the alignment voltage.
    Cal   cal_{};
    float align_vamp_ = 0.0f;
};
