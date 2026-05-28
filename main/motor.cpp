#include "motor.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <numbers>
#include <optional>
#include <utility>

#include "current_sense.h"

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "sdkconfig.h"

namespace {

constexpr const char *TAG = "motor";

constexpr int PWM_RESOLUTION_HZ = 80000000;      // 80 MHz timer tick
// In UP_DOWN (center-aligned) mode the IDF driver internally sets peak_ticks = period_ticks/2.
// One full PWM cycle takes 2 * peak_ticks ticks. To get 20 kHz: period_ticks = 80e6 / 20e3 = 4000,
// peak = 2000, compare values must be in [0, peak].
constexpr int PWM_PERIOD_TICKS = 4000;
constexpr int PWM_PEAK_TICKS   = PWM_PERIOD_TICKS / 2;
constexpr int PWM_HALF_DUTY    = PWM_PEAK_TICKS / 2;

constexpr int   COMMUTATION_PERIOD_MS = 1;
constexpr float MAX_VOLTAGE_AMP = CONFIG_MOTOR_MAX_VOLTAGE_X10 / 10.0f;
constexpr float MAX_VELOCITY    = static_cast<float>(CONFIG_MOTOR_MAX_VELOCITY_RAD_S);
constexpr float SLEW_RAD_S2     = CONFIG_MOTOR_SLEW_RAD_S2_X10 / 10.0f;
constexpr float VBUS_VOLTS      = static_cast<float>(CONFIG_MOTOR_VBUS_VOLTS);

constexpr float TWO_PI        = 2.0f * std::numbers::pi_v<float>;

constexpr float SQRT3         = std::numbers::sqrt3_v<float>;
constexpr float INV_SQRT3     = std::numbers::inv_sqrt3_v<float>;
constexpr float SQRT3_OVER_2  = SQRT3 / 2.0f;

// Fixed-point commutation (ISR side). The phase accumulator is uint32 spanning
// a full electrical revolution, so a 120° offset is exactly 2^32/3.
constexpr uint32_t PHASE_OFFSET_120 = 0x55555555u;   // +120° (2^32/3)
constexpr uint32_t PHASE_OFFSET_240 = 0xAAAAAAAAu;   // +240° (2·2^32/3)
// 3rd-harmonic injection gain (1/6) in Q15: 32768/6 ≈ 5461.
constexpr int32_t  THIRD_HARMONIC_GAIN_Q15 = 5461;
// Q15 scale and the amplitude headroom that keeps the injected waveform
// (peak ≈ √3/2 of the fundamental) from clipping the duty.
constexpr float    Q15_SCALE   = 32768.0f;
constexpr float    MAX_MOD_AMP = 0.55f;

// On enable we hold angle=0 and velocity=0 for this many ticks at the
// computed align_vamp_ so the rotor parks at the commanded electrical
// angle. Without this, starting from an arbitrary rotor position usually
// fails to sync at non-trivial commanded velocities — the field zips past
// and the rotor stays cogged in place.
constexpr uint32_t ALIGN_TICKS = 200;
// Target current through the B-C winding pair during alignment. 1 A gives
// firm parking torque without overheating a small motor for 200 ms.
constexpr float ALIGN_TARGET_CURRENT_A = 1.0f;

// Stall detection. Open-loop V/Hz can't recover from pull-out: if the rotor
// stops while the field keeps rotating, large DC currents build through the
// stuck windings. The trip threshold is runtime-configurable; the arming
// velocity is fixed because at low speeds normal operation pulls multi-amp
// peaks from v_offset that would false-trip.
constexpr int   STALL_DURATION_TICKS     = 100;   // 100 ms at 1 kHz
constexpr float STALL_ARM_VELOCITY_RAD_S = 200.0f;

// Slow EMA on |I| for stable diagnostic readout and stall trip-on-trend.
// alpha = 1/200 → ~200 ms time constant at 1 kHz; smooths the 1/√2-style
// magnitude jitter caused by bias offsets without losing real overcurrent.
constexpr float I_MAG_EMA_ALPHA = 0.005f;

// Fraction of the final value an RC/RL step reaches after one time constant:
// i(τ) = i_final · (1 − 1/e). Used to extract τ from the L-step response.
constexpr float ONE_TIME_CONSTANT_FRACTION = 1.0f - 1.0f / std::numbers::e_v<float>;

// Clarke transform → magnitude of the current vector in stator frame. For
// balanced 3-phase sinusoids this equals the peak phase current. Used by
// stall detection only; no DC subtraction so a fresh bias zero (taken at
// enable) is important for the magnitude to be meaningful.
float compute_i_mag(float ia, float ib) {
    float i_alpha = ia;
    float i_beta  = (ia + 2.0f * ib) * INV_SQRT3;
    return std::sqrt(i_alpha * i_alpha + i_beta * i_beta);
}

// Minimal scope guard: runs the supplied callable on scope exit. Lets
// identify() declare its bridge-teardown (gate off + clear the ident flag)
// once and have it run on every return path, success or error.
template <typename F>
class ScopeExit {
 public:
    explicit ScopeExit(F f) : f_(std::move(f)) {}
    ~ScopeExit() { f_(); }
    ScopeExit(const ScopeExit &) = delete;
    ScopeExit &operator=(const ScopeExit &) = delete;

 private:
    F f_;
};

}  // namespace

Motor::Motor(CurrentSense &current_sense) : current_sense_(current_sense) {}

void Motor::write_duties(float da, float db, float dc) {
    mcpwm_comparator_set_compare_value(cmp_[0], static_cast<uint32_t>(da * PWM_PEAK_TICKS));
    mcpwm_comparator_set_compare_value(cmp_[1], static_cast<uint32_t>(db * PWM_PEAK_TICKS));
    mcpwm_comparator_set_compare_value(cmp_[2], static_cast<uint32_t>(dc * PWM_PEAK_TICKS));
}

// IRAM: write three pre-clamped compare values. Called from the carrier ISR, so
// it must stay resident with the cache off (mcpwm_comparator_set_compare_value is
// in IRAM via CONFIG_MCPWM_CTRL_FUNC_IN_IRAM).
void IRAM_ATTR Motor::write_duties_ticks(uint32_t a, uint32_t b, uint32_t c) {
    mcpwm_comparator_set_compare_value(cmp_[0], a);
    mcpwm_comparator_set_compare_value(cmp_[1], b);
    mcpwm_comparator_set_compare_value(cmp_[2], c);
}

// IRAM, integer-only (no FPU — unsafe in an ISR on ESP32). Produces the compare
// value for one phase: wave = sin(θ) + (1/6)·sin(3θ) in Q15, scaled by the Q15
// amplitude and centred at 50% duty. Intermediates widened to int32/int64 so the
// multiply can't overflow; result clamped to [0, PWM_PEAK_TICKS] because
// mcpwm_comparator_set_compare_value silently rejects an out-of-range value.
uint32_t IRAM_ATTR Motor::phase_duty_ticks(uint32_t phase, int32_t amp_q) const {
    static_assert(kSinLutSize == 256, "phase>>24 index assumes a 256-entry LUT");
    uint32_t idx  = (phase >> 24) & (kSinLutSize - 1);
    uint32_t idx3 = ((phase * 3u) >> 24) & (kSinLutSize - 1);
    int32_t  s    = sin_lut_q15_[idx];
    int32_t  s3   = sin_lut_q15_[idx3];
    int32_t  wave = s + ((s3 * THIRD_HARMONIC_GAIN_Q15) >> 15);   // Q15, peak ~38230 > int16
    int32_t  scaled = static_cast<int32_t>((static_cast<int64_t>(amp_q) * wave) >> 15);
    int32_t  cmp  = PWM_HALF_DUTY + ((scaled * PWM_PEAK_TICKS) >> 15);
    if (cmp < 0)              cmp = 0;
    if (cmp > PWM_PEAK_TICKS) cmp = PWM_PEAK_TICKS;
    return static_cast<uint32_t>(cmp);
}

void Motor::set_gate(bool on) {
    gpio_set_level(static_cast<gpio_num_t>(CONFIG_MOTOR_ENABLE_GPIO), on ? 1 : 0);
}

// Task-side float→fixed conversion of the commutation command. Float math is
// fine here (task context); the ISR only ever reads the integer results.
void Motor::publish_commutation(float velocity_rad_s, float amp_volts) {
    // DDS increment: electrical revolutions per control tick mapped onto the full
    // uint32 phase span. tick rate follows the decimation. Compute in double and
    // truncate to uint32 — the low-32-bit wrap is exactly the phase wrap, and it
    // stays correct (and signed for reverse rotation) at any velocity.
    double tick_rate_hz = 20000.0 / static_cast<double>(
        commutation_decimation_.load(std::memory_order_relaxed));
    double rev_per_tick = static_cast<double>(velocity_rad_s) /
                          (2.0 * std::numbers::pi_v<double> * tick_rate_hz);
    int64_t inc = llrint(rev_per_tick * 4294967296.0);   // × 2^32
    phase_inc_.store(static_cast<uint32_t>(inc), std::memory_order_relaxed);

    // Q15 modulation amplitude, clamped to the 3rd-harmonic headroom.
    float amp = std::clamp(amp_volts / VBUS_VOLTS, 0.0f, MAX_MOD_AMP);
    amp_q_.store(static_cast<int32_t>(lrintf(amp * Q15_SCALE)), std::memory_order_relaxed);
}

// Carrier-peak ISR — now the commutation engine. Runs in IRAM and (with
// CONFIG_MCPWM_ISR_CACHE_SAFE) keeps firing while the flash cache is disabled, so
// the field keeps rotating regardless of WiFi/web/flash activity on either core.
// Integer-only: float in an ISR is UB on ESP32 (the FPU isn't saved across
// interrupts). The task publishes phase_inc_/amp_q_; here we just integrate the
// angle and emit duties — if the task stalls, the last published rate holds.
bool IRAM_ATTR Motor::on_pwm_peak(mcpwm_timer_handle_t,
                                  const mcpwm_timer_event_data_t *,
                                  void *user_ctx) {
    Motor *self = static_cast<Motor *>(user_ctx);
    BaseType_t hpw = pdFALSE;

    // Decimate the 20 kHz carrier to the commutation tick rate.
    if (++self->isr_decim_ < self->commutation_decimation_.load(std::memory_order_relaxed)) {
        return false;
    }
    self->isr_decim_ = 0;

    // identify() drives the bridge from the task (other core). Release the bridge,
    // acknowledge so identify() can start without a dual-core compare-write race,
    // and still wake the task for diagnostics.
    if (self->ident_in_progress_.load(std::memory_order_acquire)) {
        self->isr_idle_for_ident_.store(true, std::memory_order_release);
        xSemaphoreGiveFromISR(self->tick_sem_, &hpw);
        return hpw == pdTRUE;
    }

    if (self->isr_enabled_.load(std::memory_order_relaxed)) {
        // Park the phase at 0 on the enable edge (the task bumps the epoch).
        uint32_t epoch = self->phase_epoch_.load(std::memory_order_acquire);
        if (epoch != self->isr_last_epoch_) {
            self->isr_phase_      = 0;
            self->isr_last_epoch_ = epoch;
        }
        self->isr_phase_ += self->phase_inc_.load(std::memory_order_relaxed);
        int32_t  amp_q = self->amp_q_.load(std::memory_order_relaxed);
        uint32_t ph    = self->isr_phase_;
        self->write_duties_ticks(self->phase_duty_ticks(ph,                    amp_q),
                                 self->phase_duty_ticks(ph + PHASE_OFFSET_120, amp_q),
                                 self->phase_duty_ticks(ph + PHASE_OFFSET_240, amp_q));
    } else {
        // Bridge idle: 50% on all three → no winding differential.
        self->write_duties_ticks(PWM_HALF_DUTY, PWM_HALF_DUTY, PWM_HALF_DUTY);
        self->isr_phase_ = 0;
    }

    // Wake the task for ADC sampling / slew / stall detection (best-effort).
    xSemaphoreGiveFromISR(self->tick_sem_, &hpw);
    return hpw == pdTRUE;
}

void Motor::setup_mcpwm() {
    mcpwm_timer_config_t timer_cfg = {};
    timer_cfg.group_id = 0;
    timer_cfg.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_cfg.resolution_hz = PWM_RESOLUTION_HZ;
    timer_cfg.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN;
    timer_cfg.period_ticks = PWM_PERIOD_TICKS;
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &timer_));

    const std::array<int, 3> gpio = {
        CONFIG_MOTOR_PWM_A_GPIO,
        CONFIG_MOTOR_PWM_B_GPIO,
        CONFIG_MOTOR_PWM_C_GPIO,
    };

    for (int i = 0; i < 3; i++) {
        mcpwm_operator_config_t oper_cfg = {};
        oper_cfg.group_id = 0;
        ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &oper_[i]));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper_[i], timer_));

        mcpwm_comparator_config_t cmp_cfg = {};
        cmp_cfg.flags.update_cmp_on_tez = true;
        ESP_ERROR_CHECK(mcpwm_new_comparator(oper_[i], &cmp_cfg, &cmp_[i]));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmp_[i], PWM_HALF_DUTY));

        mcpwm_generator_config_t gen_cfg = {};
        gen_cfg.gen_gpio_num = gpio[i];
        ESP_ERROR_CHECK(mcpwm_new_generator(oper_[i], &gen_cfg, &gen_[i]));

        // Center-aligned: on count-up compare match drive low; on count-down compare match drive high.
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
            gen_[i],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmp_[i], MCPWM_GEN_ACTION_LOW)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
            gen_[i],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, cmp_[i], MCPWM_GEN_ACTION_HIGH)));
    }

    // Wake the motor task once per carrier peak (decimated to 1 kHz in the ISR).
    // Must be registered while the timer is still in init state (pre-enable).
    mcpwm_timer_event_callbacks_t cbs = {};
    cbs.on_full = &Motor::on_pwm_peak;
    ESP_ERROR_CHECK(mcpwm_timer_register_event_callbacks(timer_, &cbs, this));

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer_));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer_, MCPWM_TIMER_START_NO_STOP));
}

void Motor::mcpwm_init_main(void *arg) {
    // Pinned to core 1 so mcpwm_timer_register_event_callbacks allocates the
    // peripheral ISR on core 1, isolating it from WiFi activity on core 0.
    Motor *self = static_cast<Motor *>(arg);
    self->setup_mcpwm();
    xSemaphoreGive(self->mcpwm_init_done_);
    vTaskDelete(nullptr);
}

void Motor::task_main(void *arg) {
    static_cast<Motor *>(arg)->run();
}

void Motor::run() {
    float dt = COMMUTATION_PERIOD_MS / 1000.0f;

    // The electrical angle now lives in the carrier ISR (isr_phase_); the task
    // only commands rate + amplitude. It keeps the velocity slew here.
    float current_velocity = 0.0f;
    uint32_t tick_count = 0;
    bool prev_enabled = false;
    uint32_t prev_decim = commutation_decimation_.load(std::memory_order_relaxed);

    // Per-enable startup state. Counts down each tick during the ALIGN phase.
    uint32_t align_remaining = 0;
    // Stall detection counter; ticks up while overcurrent persists.
    uint32_t stall_count = 0;
    // Slow EMA on |I|, persisted across ticks.
    float i_mag_filtered = 0.0f;

    while (true) {
        // Block until the MCPWM on_full ISR signals (PWM-synchronized).
        xSemaphoreTake(tick_sem_, portMAX_DELAY);

        // Sample current sensors at the PWM-synced moment so the INA240
        // outputs land at the ripple midpoint. Useful for /motor diagnostics
        // even though no control loop reads them anymore.
        current_sense_.sample_sync();

        // While identify() owns the bridge, stay out of its way.
        if (ident_in_progress_.load(std::memory_order_acquire)) {
            continue;
        }

        // identify() may have changed the decimation; recompute the timestep so
        // wall-clock semantics stay correct.
        uint32_t decim = commutation_decimation_.load(std::memory_order_relaxed);
        if (decim != prev_decim) {
            dt = decim * (1.0f / 20000.0f);
            prev_decim = decim;
        }

        // Per-tick so a runtime slew change (POST /motor) takes effect immediately.
        float slew_per_step = slew_rad_s2_.load(std::memory_order_relaxed) * dt;

        bool enabled = enabled_.load(std::memory_order_relaxed);

        // Enable edge: start alignment phase. Bump the epoch so the ISR parks
        // the electrical angle at 0 (it holds there while phase_inc stays 0).
        if (enabled && !prev_enabled) {
            align_remaining = ALIGN_TICKS;
            current_velocity = 0.0f;
            phase_epoch_.fetch_add(1, std::memory_order_release);
        }
        prev_enabled = enabled;

        // Velocity slew. Held at 0 during ALIGN so the field doesn't move
        // before the rotor has parked.
        float target = target_velocity_rad_s_.load(std::memory_order_relaxed);
        if (align_remaining > 0) {
            current_velocity = 0.0f;
            align_remaining--;
        } else if (target > current_velocity + slew_per_step) {
            current_velocity += slew_per_step;
        } else if (target < current_velocity - slew_per_step) {
            current_velocity -= slew_per_step;
        } else {
            current_velocity = target;
        }
        current_velocity_rad_s_.store(current_velocity, std::memory_order_relaxed);

        // Read currents and magnitude once per tick — used by stall detection,
        // the bus-current estimate, and the periodic debug log.
        CurrentSense::Phases ph = current_sense_.read();
        float i_mag_raw = compute_i_mag(ph.ia, ph.ib);

        // Slow EMA on |I| smooths the bias-induced jitter (range ~1 A) so the
        // trip decision is made on a steady value, not transient peaks.
        i_mag_filtered += I_MAG_EMA_ALPHA * (i_mag_raw - i_mag_filtered);
        i_mag_filtered_a_.store(i_mag_filtered, std::memory_order_relaxed);

        // Bus-current estimate. For a balanced 3-phase VSI with power factor 1,
        // I_bus = (3/2) * V_phase_peak * I_phase_peak / V_bus. At sync the
        // power factor is < 1 so this overestimates; at stall PF ≈ 1 so it's
        // accurate. Either way it correlates with what a bench supply reports.
        float vamp_last = voltage_amplitude_v_.load(std::memory_order_relaxed);
        float i_bus_est = 1.5f * vamp_last * i_mag_filtered / VBUS_VOLTS;
        i_bus_est_a_.store(i_bus_est, std::memory_order_relaxed);

        // Stall detection: only active when running steadily past ALIGN above
        // the arming velocity. Uses filtered magnitude so transient peaks
        // (from bias jitter) don't trip.
        float stall_threshold = stall_current_a_.load(std::memory_order_relaxed);
        if (enabled && align_remaining == 0 &&
            std::fabs(current_velocity) > STALL_ARM_VELOCITY_RAD_S) {
            if (i_mag_filtered > stall_threshold) {
                if (++stall_count > STALL_DURATION_TICKS) {
                    set_gate(false);
                    enabled_.store(false, std::memory_order_relaxed);
                    isr_enabled_.store(false, std::memory_order_relaxed);
                    phase_inc_.store(0, std::memory_order_relaxed);
                    amp_q_.store(0, std::memory_order_relaxed);
                    stalled_.store(true, std::memory_order_relaxed);
                    ESP_LOGE(TAG, "stall: |I|=%.2f A > %.2f A for >%d ms — disabling",
                        static_cast<double>(i_mag_filtered), static_cast<double>(stall_threshold),
                        STALL_DURATION_TICKS);
                    stall_count = 0;
                    enabled = false;
                }
            } else {
                stall_count = 0;
            }
        } else {
            stall_count = 0;
        }

        float vamp;
        if (enabled) {
            // V/Hz mapping. During ALIGN use a cal-derived voltage that
            // produces a known parking current independent of the user's
            // v_offset (which may be tuned for spinning operation and would
            // be unsafe at standstill).
            if (align_remaining > 0 && cal_.valid) {
                vamp = align_vamp_;
            } else {
                float v_off = v_offset_v_.load(std::memory_order_relaxed);
                float v_per = v_per_rad_s_.load(std::memory_order_relaxed);
                vamp = v_off + v_per * std::fabs(current_velocity);
            }
            vamp = std::clamp(vamp, 0.0f, MAX_VOLTAGE_AMP);

            // Hand the rate + amplitude to the carrier ISR. The ISR does the
            // 3rd-harmonic-injected 3-phase synthesis (in integer Q15) and the
            // compare writes; if this task stalls, the ISR keeps rotating the
            // field at the last published rate.
            publish_commutation(current_velocity, vamp);
        } else {
            // Bridge idle. isr_enabled_ is already false (set by disable()/stall),
            // so the ISR holds all three phases at 50% — no winding differential.
            vamp = 0.0f;
            current_velocity = 0.0f;
        }
        voltage_amplitude_v_.store(vamp, std::memory_order_relaxed);

        tick_count++;
        uptime_s_.store(tick_count / 1000, std::memory_order_relaxed);
    }
}

esp_err_t Motor::init() {
    for (int i = 0; i < kSinLutSize; i++) {
        float s = std::sin(TWO_PI * static_cast<float>(i) / static_cast<float>(kSinLutSize));
        sin_lut_q15_[i] = static_cast<int16_t>(std::lrintf(s * 32767.0f));   // Q15
    }

    // Compile-time default; NVS / POST /motor may override at runtime.
    slew_rad_s2_.store(SLEW_RAD_S2, std::memory_order_relaxed);

    tick_sem_ = xSemaphoreCreateBinary();
    if (!tick_sem_) {
        ESP_LOGE(TAG, "failed to create tick semaphore");
        return ESP_ERR_NO_MEM;
    }

    // Allocate the MCPWM ISR on core 1 (same core as the motor task) so WiFi
    // traffic on core 0 cannot delay or drop carrier-peak interrupts.
    mcpwm_init_done_ = xSemaphoreCreateBinary();
    if (!mcpwm_init_done_) {
        ESP_LOGE(TAG, "failed to create mcpwm init semaphore");
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreatePinnedToCore(&Motor::mcpwm_init_main, "mcpwm_init", 4096, this,
                                configMAX_PRIORITIES - 1, nullptr, 1) != pdPASS) {
        ESP_LOGE(TAG, "failed to spawn mcpwm init task");
        return ESP_FAIL;
    }
    xSemaphoreTake(mcpwm_init_done_, portMAX_DELAY);
    vSemaphoreDelete(mcpwm_init_done_);
    mcpwm_init_done_ = nullptr;

    gpio_config_t enable_cfg = {};
    enable_cfg.pin_bit_mask = 1ULL << CONFIG_MOTOR_ENABLE_GPIO;
    enable_cfg.mode = GPIO_MODE_OUTPUT;
    enable_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    enable_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    enable_cfg.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&enable_cfg));
    set_gate(false);

    write_duties(0.5f, 0.5f, 0.5f);

    BaseType_t ok = xTaskCreatePinnedToCore(&Motor::task_main, "motor", 4096, this, 5, nullptr, 1);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to spawn motor task");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "init ok: A=%d B=%d C=%d SD=%d, carrier=%d Hz, Vbus=%.1f V",
        CONFIG_MOTOR_PWM_A_GPIO, CONFIG_MOTOR_PWM_B_GPIO, CONFIG_MOTOR_PWM_C_GPIO,
        CONFIG_MOTOR_ENABLE_GPIO,
        PWM_RESOLUTION_HZ / PWM_PERIOD_TICKS,
        static_cast<double>(VBUS_VOLTS));
    return ESP_OK;
}

void Motor::enable() {
    // Re-zero current sensor bias with the gate still off. The INA240+shunt
    // drifts with temperature, so a fresh zero is needed for stall detection
    // to have accurate readings.
    current_sense_.calibrate_bias();
    stalled_.store(false);
    set_gate(true);
    // Hand commutation to the ISR. phase_inc_/amp_q_ are 0 (cleared at disable),
    // so the field stays parked until run() detects the edge and publishes the
    // alignment command — no stale rate leaks in during the handoff.
    isr_enabled_.store(true, std::memory_order_relaxed);
    enabled_.store(true);
    ESP_LOGI(TAG, "enabled");
}

void Motor::disable() {
    // Soft stop: ramp the commanded velocity to 0 before cutting the gate.
    // Abruptly tristate-ing the bridge with a spinning rotor lets the
    // back-EMF freewheel through body diodes and emits a switching transient
    // that couples noise into nearby signals (e.g. the debug UART). Waiting
    // for the task's existing slew loop to bring current_velocity to ~0 means
    // the rotor is at rest against the static field by the time the gate
    // drops — no transient, no noise. Bounded so a stuck velocity can't keep
    // the gate live forever; stall trip in run() bypasses this and cuts now.
    if (enabled_.load()) {
        target_velocity_rad_s_.store(0.0f);
        const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(5000);
        while (std::fabs(current_velocity_rad_s_.load()) > 1.0f &&
               xTaskGetTickCount() < deadline) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        float v_left = current_velocity_rad_s_.load();
        if (std::fabs(v_left) > 1.0f) {
            ESP_LOGW(TAG, "soft-stop timed out at %.1f rad/s; cutting gate hard",
                static_cast<double>(v_left));
        }
    }
    enabled_.store(false);
    isr_enabled_.store(false, std::memory_order_relaxed);
    phase_inc_.store(0, std::memory_order_relaxed);
    amp_q_.store(0, std::memory_order_relaxed);
    set_gate(false);
    ESP_LOGI(TAG, "disabled");
}

void Motor::set_velocity(float rad_per_sec) {
    target_velocity_rad_s_.store(std::clamp(rad_per_sec, -MAX_VELOCITY, MAX_VELOCITY));
}

void Motor::set_voltage_amplitude(float volts) {
    v_offset_v_.store(std::clamp(volts, 0.0f, MAX_VOLTAGE_AMP));
}

void Motor::set_v_per_rad_s(float v_per_rad_s) {
    v_per_rad_s_.store(std::max(v_per_rad_s, 0.0f));
}

void Motor::set_stall_current_a(float amps) {
    // Floor at zero; setting it too low (< noise floor) would make the detector
    // trip on idle bias jitter. Zero means "disabled".
    stall_current_a_.store(std::max(amps, 0.0f));
}

void Motor::set_slew_rad_s2(float rad_s2) {
    // Clamp to the Kconfig range (1..100000 ×10 ⇒ 0.1..10000 rad/s²). A floor
    // keeps the ramp moving; too-high invites open-loop pull-out (user's risk).
    slew_rad_s2_.store(std::clamp(rad_s2, 1.0f, 10000.0f));
}

Motor::Status Motor::status() const {
    Status out;
    out.target_velocity_rad_s  = target_velocity_rad_s_.load();
    out.current_velocity_rad_s = current_velocity_rad_s_.load();
    out.voltage_amplitude_v    = voltage_amplitude_v_.load();
    out.v_offset_v             = v_offset_v_.load();
    out.v_per_rad_s            = v_per_rad_s_.load();
    out.slew_rad_s2            = slew_rad_s2_.load();
    out.i_mag_a                = i_mag_filtered_a_.load();
    out.i_bus_est_a            = i_bus_est_a_.load();
    out.stall_current_a        = stall_current_a_.load();
    out.enabled                = enabled_.load();
    out.stalled                = stalled_.load();
    out.uptime_s               = uptime_s_.load();
    return out;
}

void Motor::set_cal(const Cal &cal) {
    if (!cal.valid) return;
    cal_ = cal;
    // Align voltage solves: V_LL = sqrt(3) * vamp = V_dead + 2*Rs*I_target.
    align_vamp_ = (cal.v_dead + 2.0f * cal.rs_ohm * ALIGN_TARGET_CURRENT_A) / SQRT3;
    ESP_LOGI(TAG, "cal applied: Rs=%.4f Ω, Ls=%.6f H, V_dead=%.4f V, align_vamp=%.4f V",
        static_cast<double>(cal.rs_ohm), static_cast<double>(cal.ls_henry),
        static_cast<double>(cal.v_dead), static_cast<double>(align_vamp_));
}

Motor::Cal Motor::cal() const {
    return cal_;
}

// Drive duties directly at electrical angle 0 (current flows B↔C, A idle).
// Used during identification to apply known DC voltages. Line-to-line BC
// voltage = sqrt(3) * vamp, so amp = vbc / (sqrt(3) * Vbus).
void Motor::ident_apply_voltage_bc(float vbc) {
    float amp = std::clamp(vbc / (SQRT3 * VBUS_VOLTS), 0.0f, 0.30f);   // 0.30 = safety cap inside ID
    float da = 0.5f;
    float db = 0.5f - SQRT3_OVER_2 * amp;
    float dc = 0.5f + SQRT3_OVER_2 * amp;
    write_duties(da, db, dc);
}

std::optional<Motor::Cal> Motor::identify() {
    if (enabled_.load()) {
        ESP_LOGE(TAG, "identify refused: motor is enabled");
        return std::nullopt;
    }

    ESP_LOGI(TAG, "identify start");

    // Take the bridge from both the motor task and the carrier ISR. The 5 ms
    // delay lets the task bail; the ISR runs on the other core and never blocks,
    // so it acks via isr_idle_for_ident_ once it has stopped writing compares.
    // Wait for that ack before we drive the compares ourselves — otherwise both
    // cores write the same comparator registers (a data race).
    isr_idle_for_ident_.store(false, std::memory_order_relaxed);
    ident_in_progress_.store(true, std::memory_order_release);
    vTaskDelay(pdMS_TO_TICKS(5));
    bool isr_released = false;
    for (int i = 0; i < 50; i++) {
        if (isr_idle_for_ident_.load(std::memory_order_acquire)) { isr_released = true; break; }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (!isr_released) {
        ESP_LOGW(TAG, "identify: carrier ISR did not ack bridge release; proceeding");
    }

    // Bridge teardown for every exit path below: cut the gate and hand the
    // bridge back to the motor task. Declared once here so no early-return
    // error case can forget either step.
    ScopeExit teardown([this] {
        set_gate(false);
        ident_in_progress_.store(false, std::memory_order_release);
    });

    // Re-zero the current sensor with the gate still off — the INA240+shunt
    // drifts with temperature, so a fresh zero matches the current thermal
    // state of the board.
    current_sense_.calibrate_bias();

    // Enable the gate driver.
    set_gate(true);

    // --- Rs measurement (adaptive) ---
    //
    // We don't know R upfront; bracket-search to find a probe voltage that
    // puts the current in a comfortable band, then take 5 probes ascending
    // from there and least-squares fit V = R·I + V_dead. The intercept is
    // the gate-driver dead-band loss; the slope is the loop resistance.
    const float I_DISC_LO   = 1.5f;
    const float I_DISC_HI   = 2.5f;
    const float I_SAT_LIMIT = 5.0f;   // just under INA240 saturation (~5.8 A)
    const float V_MIN       = 0.005f;
    const float V_MAX       = 1.0f;

    float v_disc = 0.05f;
    float i_disc = 0.0f;
    float v_low  = 0.0f;   // 0 = no bracket yet
    float v_high = 0.0f;
    bool  v_found = false;
    for (int attempt = 0; attempt < 10; attempt++) {
        ident_apply_voltage_bc(v_disc);
        vTaskDelay(pdMS_TO_TICKS(80));
        i_disc = std::fabs(current_sense_.read().ib);
        ESP_LOGI(TAG, "identify Rs discover: V=%.4f, I=%.4f A", static_cast<double>(v_disc), static_cast<double>(i_disc));
        if (i_disc > I_DISC_HI) {
            ident_apply_voltage_bc(0.0f);
            vTaskDelay(pdMS_TO_TICKS(20));
            v_high = v_disc;
            v_disc = (v_low > 0.0f) ? 0.5f * (v_low + v_high) : 0.5f * v_disc;
            if (v_disc < V_MIN) break;
            continue;
        }
        if (i_disc < I_DISC_LO) {
            v_low = v_disc;
            v_disc = (v_high > 0.0f) ? 0.5f * (v_low + v_high) : 2.0f * v_disc;
            if (v_disc > V_MAX) break;
            continue;
        }
        v_found = true;
        break;
    }
    if (!v_found) {
        ident_apply_voltage_bc(0.0f);
        ESP_LOGE(TAG, "identify: could not find a usable Rs probe voltage "
                      "(last V=%.4f, I=%.4f) — check wiring and current sensor",
            static_cast<double>(v_disc), static_cast<double>(i_disc));
        return std::nullopt;
    }

    // Five ascending probes 1.0×..1.4× of v_disc.
    const std::array<float, 5> scale = {1.0f, 1.1f, 1.2f, 1.3f, 1.4f};
    std::array<float, 5> v_pts{};
    std::array<float, 5> i_pts{};
    int n_valid = 0;
    for (int i = 0; i < 5; i++) {
        float v_probe = v_disc * scale[i];
        ident_apply_voltage_bc(v_probe);
        vTaskDelay(pdMS_TO_TICKS(80));
        float i_loop = std::fabs(current_sense_.read().ib);
        ESP_LOGI(TAG, "identify Rs probe: V=%.4f, I=%.4f A", static_cast<double>(v_probe), static_cast<double>(i_loop));
        if (i_loop > I_SAT_LIMIT) break;     // past linear range — stop here
        if (i_loop < 0.3f)        continue;  // suspiciously low — skip but keep going
        v_pts[n_valid] = v_probe;
        i_pts[n_valid] = i_loop;
        n_valid++;
    }
    ident_apply_voltage_bc(0.0f);
    vTaskDelay(pdMS_TO_TICKS(30));

    if (n_valid < 3) {
        ESP_LOGE(TAG, "identify Rs: only %d valid probes — can't fit", n_valid);
        return std::nullopt;
    }

    // Least-squares linear fit V = R·I + V_dead.
    float sum_i = 0, sum_v = 0, sum_ii = 0, sum_iv = 0;
    for (int i = 0; i < n_valid; i++) {
        sum_i  += i_pts[i];
        sum_v  += v_pts[i];
        sum_ii += i_pts[i] * i_pts[i];
        sum_iv += i_pts[i] * v_pts[i];
    }
    float denom = n_valid * sum_ii - sum_i * sum_i;
    if (std::fabs(denom) < 1e-9f) {
        ESP_LOGE(TAG, "identify Rs: degenerate probe data");
        return std::nullopt;
    }
    float r_loop = (n_valid * sum_iv - sum_i * sum_v) / denom;
    float v_dead = (sum_v - r_loop * sum_i) / n_valid;
    float rs = r_loop * 0.5f;
    ESP_LOGI(TAG, "identify Rs fit: n=%d, R_loop=%.4f Ω, V_dead=%.4f V",
        n_valid, static_cast<double>(r_loop), static_cast<double>(v_dead));

    if (r_loop <= 0.0f) {
        ESP_LOGE(TAG, "identify Rs: negative slope — wiring or sensor issue");
        return std::nullopt;
    }

    // --- Ls measurement ---
    //
    // Bump decimation to 1 (20 kHz sampling) and capture the current rise
    // after a step. τ = first time at which i ≥ 0.632 · i_final.

    float v_step = v_dead + 2.0f * r_loop;   // targets i_final ≈ 2 A through BC pair
    if (v_step < 0.05f) v_step = 0.05f;
    if (v_step > 0.5f)  v_step = 0.5f;

    commutation_decimation_.store(1, std::memory_order_relaxed);
    vTaskDelay(pdMS_TO_TICKS(2));
    ESP_LOGI(TAG, "identify L step: V=%.4f (targeting i_final ~ %.2f A)",
        static_cast<double>(v_step), static_cast<double>((v_step - v_dead) / r_loop));

    const int n_samples = 200;          // 200 × 50 µs = 10 ms capture
    static float samples[200];
    int64_t t0 = esp_timer_get_time();
    ident_apply_voltage_bc(v_step);
    for (int i = 0; i < n_samples; i++) {
        int64_t deadline_us = t0 + static_cast<int64_t>(i) * 50;
        while (esp_timer_get_time() < deadline_us) { /* spin */ }
        samples[i] = std::fabs(current_sense_.read().ib);
    }
    ident_apply_voltage_bc(0.0f);
    vTaskDelay(pdMS_TO_TICKS(20));
    commutation_decimation_.store(kDecimationDefault, std::memory_order_relaxed);

    // i_final = average of last 20 samples.
    float i_final = 0.0f;
    for (int i = n_samples - 20; i < n_samples; i++) i_final += samples[i];
    i_final /= 20.0f;
    if (i_final < 0.01f) {
        ESP_LOGE(TAG, "identify L: final current %.4f A too low", static_cast<double>(i_final));
        return std::nullopt;
    }
    float i_thresh = ONE_TIME_CONSTANT_FRACTION * i_final;
    int idx_tau = -1;
    for (int i = 0; i < n_samples; i++) {
        if (samples[i] >= i_thresh) { idx_tau = i; break; }
    }
    if (idx_tau <= 0) {
        ESP_LOGE(TAG, "identify L: could not extract τ from step response");
        return std::nullopt;
    }
    float tau_s = idx_tau * 50e-6f;
    float l_loop = r_loop * tau_s;
    float ls = l_loop * 0.5f;
    ESP_LOGI(TAG, "identify L: i_final=%.4f A, idx_tau=%d, τ=%.3f ms, L_loop=%.6f H",
        static_cast<double>(i_final), idx_tau, static_cast<double>(tau_s * 1000.0f),
        static_cast<double>(l_loop));

    // Leave the bridge in a neutral state; the teardown guard cuts the gate.
    write_duties(0.5f, 0.5f, 0.5f);

    Cal result;
    result.valid    = true;
    result.rs_ohm   = rs;
    result.ls_henry = ls;
    result.v_dead   = v_dead;
    set_cal(result);

    ESP_LOGI(TAG, "identify ok: Rs=%.4f Ω, Ls=%.6f H, V_dead=%.4f V",
        static_cast<double>(rs), static_cast<double>(ls), static_cast<double>(v_dead));
    return result;
}
