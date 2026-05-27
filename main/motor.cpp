#include "motor.h"

#include <atomic>
#include <math.h>
#include <string.h>

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

using std::atomic_load;
using std::atomic_store;
using std::atomic_load_explicit;
using std::atomic_store_explicit;
using std::memory_order_relaxed;
using std::memory_order_acquire;
using std::memory_order_release;

static const char *TAG = "motor";

#define PWM_RESOLUTION_HZ 80000000      // 80 MHz timer tick
// In UP_DOWN (center-aligned) mode the IDF driver internally sets peak_ticks = period_ticks/2.
// One full PWM cycle takes 2 * peak_ticks ticks. To get 20 kHz: period_ticks = 80e6 / 20e3 = 4000,
// peak = 2000, compare values must be in [0, peak].
#define PWM_PERIOD_TICKS  4000
#define PWM_PEAK_TICKS    (PWM_PERIOD_TICKS / 2)
#define PWM_HALF_DUTY     (PWM_PEAK_TICKS / 2)

#define COMMUTATION_PERIOD_MS 1
#define MAX_VOLTAGE_AMP   (CONFIG_MOTOR_MAX_VOLTAGE_X10 / 10.0f)
#define MAX_VELOCITY      ((float)CONFIG_MOTOR_MAX_VELOCITY_RAD_S)
#define SLEW_RAD_S2       (CONFIG_MOTOR_SLEW_RAD_S2_X10 / 10.0f)
#define VBUS_VOLTS        ((float)CONFIG_MOTOR_VBUS_VOLTS)

#define SIN_LUT_SIZE      256
#define SIN_LUT_MASK      (SIN_LUT_SIZE - 1)
#define LUT_SCALE         (SIN_LUT_SIZE / (2.0f * (float)M_PI))
#define TWO_PI            (2.0f * (float)M_PI)
#define TWO_PI_OVER_3     (TWO_PI / 3.0f)

// On enable we hold angle=0 and velocity=0 for this many ticks at the
// computed s_align_vamp so the rotor parks at the commanded electrical
// angle. Without this, starting from an arbitrary rotor position usually
// fails to sync at non-trivial commanded velocities — the field zips past
// and the rotor stays cogged in place.
#define ALIGN_TICKS              200
// Target current through the B-C winding pair during alignment. 1 A gives
// firm parking torque without overheating a small motor for 200 ms.
#define ALIGN_TARGET_CURRENT_A   1.0f

// Stall detection. Open-loop V/Hz can't recover from pull-out: if the rotor
// stops while the field keeps rotating, large DC currents build through the
// stuck windings. The trip threshold is runtime-configurable; the arming
// velocity is fixed because at low speeds normal operation pulls multi-amp
// peaks from v_offset that would false-trip.
#define DEFAULT_STALL_CURRENT_A  3.0f
#define STALL_DURATION_TICKS     100    // 100 ms at 1 kHz
#define STALL_ARM_VELOCITY_RAD_S 200.0f

// Slow EMA on |I| for stable diagnostic readout and stall trip-on-trend.
// alpha = 1/200 → ~200 ms time constant at 1 kHz; smooths the 1/√2-style
// magnitude jitter caused by bias offsets without losing real overcurrent.
#define I_MAG_EMA_ALPHA          0.005f

// 20 kHz carrier divided down. Runtime variable so motor_identify can drop
// to 1 (full 20 kHz sampling) during its L-step capture.
#define COMMUTATION_DECIMATION_DEFAULT 20
static volatile uint32_t s_commutation_decimation = COMMUTATION_DECIMATION_DEFAULT;

static float s_sin_lut[SIN_LUT_SIZE];

static mcpwm_timer_handle_t s_timer;
static mcpwm_oper_handle_t  s_oper[3];
static mcpwm_cmpr_handle_t  s_cmp[3];
static mcpwm_gen_handle_t   s_gen[3];

static SemaphoreHandle_t s_tick_sem;
static SemaphoreHandle_t s_mcpwm_init_done;

// Shared state. Float / int writes to 4-byte-aligned addresses are atomic
// on Xtensa, so plain atomic_store / atomic_load is enough.
static std::atomic<float>    g_target_velocity_rad_s  = 0.0f;
static std::atomic<float>    g_current_velocity_rad_s = 0.0f;
static std::atomic<float>    g_voltage_amplitude_v    = 0.0f;
static std::atomic<float>    g_v_offset_v             = 0.0f;
static std::atomic<float>    g_v_per_rad_s            = 0.0f;
static std::atomic<float>    g_stall_current_a        = DEFAULT_STALL_CURRENT_A;
static std::atomic<float>    g_i_mag_filtered_a       = 0.0f;
static std::atomic<float>    g_i_bus_est_a            = 0.0f;
static std::atomic<bool>     g_motor_enabled          = false;
static std::atomic<uint32_t> g_uptime_s               = 0;

// motor_identify takes ownership of the bridge briefly. The motor task
// observes this flag and skips its loop body while it's set.
static std::atomic<bool>     g_ident_in_progress      = false;

// Loaded from /calibrate or NVS. Used to size the alignment voltage.
static motor_cal_t s_cal       = {};
static float       s_align_vamp = 0.0f;

// Per-enable startup state. Counts down each tick during the ALIGN phase.
static uint32_t s_align_remaining = 0;

// Stall detection state. Counter ticks up while overcurrent persists; sticky
// flag is set on trip and cleared on the next motor_enable.
static uint32_t s_stall_count = 0;
static std::atomic<bool> g_stalled  = false;

static bool IRAM_ATTR on_pwm_peak(mcpwm_timer_handle_t t,
                                  const mcpwm_timer_event_data_t *e,
                                  void *user_ctx) {
    static uint32_t decim = 0;
    BaseType_t hpw = pdFALSE;
    if (++decim >= s_commutation_decimation) {
        decim = 0;
        xSemaphoreGiveFromISR(s_tick_sem, &hpw);
    }
    return hpw == pdTRUE;
}

static void sin_lut_init(void) {
    for (int i = 0; i < SIN_LUT_SIZE; i++) {
        s_sin_lut[i] = sinf(TWO_PI * (float)i / (float)SIN_LUT_SIZE);
    }
}

static inline float fast_sin(float angle_rad) {
    while (angle_rad < 0.0f)    angle_rad += TWO_PI;
    while (angle_rad >= TWO_PI) angle_rad -= TWO_PI;
    int idx = (int)(angle_rad * LUT_SCALE);
    return s_sin_lut[idx & SIN_LUT_MASK];
}

static inline float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// Clarke transform → magnitude of the current vector in stator frame. For
// balanced 3-phase sinusoids this equals the peak phase current. Used by
// stall detection only; no DC subtraction so a fresh bias zero (taken at
// motor_enable) is important for the magnitude to be meaningful.
static inline float compute_i_mag(float ia, float ib) {
    float i_alpha = ia;
    float i_beta  = (ia + 2.0f * ib) * 0.5773502691f;   // 1/sqrt(3)
    return sqrtf(i_alpha * i_alpha + i_beta * i_beta);
}

static inline void write_duties(float da, float db, float dc) {
    mcpwm_comparator_set_compare_value(s_cmp[0], (uint32_t)(da * PWM_PEAK_TICKS));
    mcpwm_comparator_set_compare_value(s_cmp[1], (uint32_t)(db * PWM_PEAK_TICKS));
    mcpwm_comparator_set_compare_value(s_cmp[2], (uint32_t)(dc * PWM_PEAK_TICKS));
}

static void setup_mcpwm(void) {
    mcpwm_timer_config_t timer_cfg = {};
    timer_cfg.group_id = 0;
    timer_cfg.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_cfg.resolution_hz = PWM_RESOLUTION_HZ;
    timer_cfg.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN;
    timer_cfg.period_ticks = PWM_PERIOD_TICKS;
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &s_timer));

    const int gpio[3] = {
        CONFIG_MOTOR_PWM_A_GPIO,
        CONFIG_MOTOR_PWM_B_GPIO,
        CONFIG_MOTOR_PWM_C_GPIO,
    };

    for (int i = 0; i < 3; i++) {
        mcpwm_operator_config_t oper_cfg = {};
        oper_cfg.group_id = 0;
        ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &s_oper[i]));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(s_oper[i], s_timer));

        mcpwm_comparator_config_t cmp_cfg = {};
        cmp_cfg.flags.update_cmp_on_tez = true;
        ESP_ERROR_CHECK(mcpwm_new_comparator(s_oper[i], &cmp_cfg, &s_cmp[i]));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(s_cmp[i], PWM_HALF_DUTY));

        mcpwm_generator_config_t gen_cfg = {};
        gen_cfg.gen_gpio_num = gpio[i];
        ESP_ERROR_CHECK(mcpwm_new_generator(s_oper[i], &gen_cfg, &s_gen[i]));

        // Center-aligned: on count-up compare match drive low; on count-down compare match drive high.
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
            s_gen[i],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s_cmp[i], MCPWM_GEN_ACTION_LOW)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
            s_gen[i],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, s_cmp[i], MCPWM_GEN_ACTION_HIGH)));
    }

    // Wake the motor task once per carrier peak (decimated to 1 kHz in the ISR).
    // Must be registered while the timer is still in init state (pre-enable).
    mcpwm_timer_event_callbacks_t cbs = {};
    cbs.on_full = on_pwm_peak;
    ESP_ERROR_CHECK(mcpwm_timer_register_event_callbacks(s_timer, &cbs, NULL));

    ESP_ERROR_CHECK(mcpwm_timer_enable(s_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(s_timer, MCPWM_TIMER_START_NO_STOP));
}

static void mcpwm_init_task(void *arg) {
    // Pinned to core 1 so mcpwm_timer_register_event_callbacks allocates the
    // peripheral ISR on core 1, isolating it from WiFi activity on core 0.
    setup_mcpwm();
    xSemaphoreGive(s_mcpwm_init_done);
    vTaskDelete(NULL);
}

static void motor_task(void *arg) {
    float dt = COMMUTATION_PERIOD_MS / 1000.0f;
    float slew_per_step = SLEW_RAD_S2 * dt;

    float angle = 0.0f;
    float current_velocity = 0.0f;
    uint32_t tick_count = 0;
    bool prev_enabled = false;
    uint32_t prev_decim = s_commutation_decimation;

    while (1) {
        // Block until the MCPWM on_full ISR signals (PWM-synchronized).
        xSemaphoreTake(s_tick_sem, portMAX_DELAY);

        // Sample current sensors at the PWM-synced moment so the INA240
        // outputs land at the ripple midpoint. Useful for /motor diagnostics
        // even though no control loop reads them anymore.
        current_sense_sample_sync();

        // While motor_identify owns the bridge, stay out of its way.
        if (atomic_load_explicit(&g_ident_in_progress, memory_order_acquire)) {
            continue;
        }

        // motor_identify may have changed the decimation; recompute timestep
        // and slew step so wall-clock semantics stay correct.
        uint32_t decim = s_commutation_decimation;
        if (decim != prev_decim) {
            dt = decim * (1.0f / 20000.0f);
            slew_per_step = SLEW_RAD_S2 * dt;
            prev_decim = decim;
        }

        bool enabled = atomic_load_explicit(&g_motor_enabled, memory_order_relaxed);

        // Enable edge: start alignment phase. Rotor parks at angle 0.
        if (enabled && !prev_enabled) {
            s_align_remaining = ALIGN_TICKS;
            angle = 0.0f;
            current_velocity = 0.0f;
        }
        prev_enabled = enabled;

        // Velocity slew. Held at 0 during ALIGN so the field doesn't move
        // before the rotor has parked.
        float target = atomic_load_explicit(&g_target_velocity_rad_s, memory_order_relaxed);
        if (s_align_remaining > 0) {
            current_velocity = 0.0f;
            s_align_remaining--;
        } else if (target > current_velocity + slew_per_step) {
            current_velocity += slew_per_step;
        } else if (target < current_velocity - slew_per_step) {
            current_velocity -= slew_per_step;
        } else {
            current_velocity = target;
        }
        atomic_store_explicit(&g_current_velocity_rad_s, current_velocity, memory_order_relaxed);

        // Read currents and magnitude once per tick — used by stall detection,
        // the bus-current estimate, and the periodic debug log.
        float ia, ib, ic;
        current_sense_read(&ia, &ib, &ic);
        float i_mag_raw = compute_i_mag(ia, ib);

        // Slow EMA on |I| smooths the bias-induced jitter (range ~1 A) so the
        // trip decision is made on a steady value, not transient peaks.
        static float i_mag_filtered = 0.0f;
        i_mag_filtered += I_MAG_EMA_ALPHA * (i_mag_raw - i_mag_filtered);
        atomic_store_explicit(&g_i_mag_filtered_a, i_mag_filtered, memory_order_relaxed);

        // Bus-current estimate. For a balanced 3-phase VSI with power factor 1,
        // I_bus = (3/2) * V_phase_peak * I_phase_peak / V_bus. At sync the
        // power factor is < 1 so this overestimates; at stall PF ≈ 1 so it's
        // accurate. Either way it correlates with what a bench supply reports.
        float vamp_last = atomic_load_explicit(&g_voltage_amplitude_v, memory_order_relaxed);
        float i_bus_est = 1.5f * vamp_last * i_mag_filtered / VBUS_VOLTS;
        atomic_store_explicit(&g_i_bus_est_a, i_bus_est, memory_order_relaxed);

        // Stall detection: only active when running steadily past ALIGN above
        // the arming velocity. Uses filtered magnitude so transient peaks
        // (from bias jitter) don't trip.
        float stall_threshold = atomic_load_explicit(&g_stall_current_a, memory_order_relaxed);
        if (enabled && s_align_remaining == 0 &&
            fabsf(current_velocity) > STALL_ARM_VELOCITY_RAD_S) {
            if (i_mag_filtered > stall_threshold) {
                if (++s_stall_count > STALL_DURATION_TICKS) {
                    gpio_set_level((gpio_num_t)CONFIG_MOTOR_ENABLE_GPIO, 0);
                    atomic_store_explicit(&g_motor_enabled, false, memory_order_relaxed);
                    atomic_store_explicit(&g_stalled, true, memory_order_relaxed);
                    ESP_LOGE(TAG, "stall: |I|=%.2f A > %.2f A for >%d ms — disabling",
                        (double)i_mag_filtered, (double)stall_threshold,
                        STALL_DURATION_TICKS);
                    s_stall_count = 0;
                    enabled = false;
                }
            } else {
                s_stall_count = 0;
            }
        } else {
            s_stall_count = 0;
        }

        float vamp;
        if (enabled) {
            angle += current_velocity * dt;
            if (angle >= TWO_PI) angle -= TWO_PI;
            if (angle < 0.0f)    angle += TWO_PI;

            // V/Hz mapping. During ALIGN use a cal-derived voltage that
            // produces a known parking current independent of the user's
            // v_offset (which may be tuned for spinning operation and would
            // be unsafe at standstill).
            if (s_align_remaining > 0 && s_cal.valid) {
                vamp = s_align_vamp;
            } else {
                float v_off = atomic_load_explicit(&g_v_offset_v, memory_order_relaxed);
                float v_per = atomic_load_explicit(&g_v_per_rad_s, memory_order_relaxed);
                vamp = v_off + v_per * fabsf(current_velocity);
            }
            vamp = clampf(vamp, 0.0f, MAX_VOLTAGE_AMP);

            float amp = vamp / VBUS_VOLTS;
            // 3rd-harmonic injection: common-mode signal that does not appear
            // in the line-to-line voltage the motor sees, but flattens the
            // per-phase waveform so we can run with a higher fundamental
            // amplitude (up to ~0.577 of Vbus) without clipping the duty.
            amp = clampf(amp, 0.0f, 0.55f);
            float third = (1.0f / 6.0f) * fast_sin(3.0f * angle);

            float da = 0.5f + amp * (fast_sin(angle)                 + third);
            float db = 0.5f + amp * (fast_sin(angle - TWO_PI_OVER_3) + third);
            float dc = 0.5f + amp * (fast_sin(angle + TWO_PI_OVER_3) + third);
            write_duties(da, db, dc);
        } else {
            // Bridge idle: all three at 50% means no winding differential.
            vamp = 0.0f;
            angle = 0.0f;
            current_velocity = 0.0f;
            write_duties(0.5f, 0.5f, 0.5f);
        }
        atomic_store_explicit(&g_voltage_amplitude_v, vamp, memory_order_relaxed);

        tick_count++;
        atomic_store_explicit(&g_uptime_s, tick_count / 1000, memory_order_relaxed);
    }
}

esp_err_t motor_init(void) {
    sin_lut_init();

    s_tick_sem = xSemaphoreCreateBinary();
    if (!s_tick_sem) {
        ESP_LOGE(TAG, "failed to create tick semaphore");
        return ESP_ERR_NO_MEM;
    }

    // Allocate the MCPWM ISR on core 1 (same core as motor_task) so WiFi
    // traffic on core 0 cannot delay or drop carrier-peak interrupts.
    s_mcpwm_init_done = xSemaphoreCreateBinary();
    if (!s_mcpwm_init_done) {
        ESP_LOGE(TAG, "failed to create mcpwm init semaphore");
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreatePinnedToCore(mcpwm_init_task, "mcpwm_init", 4096, NULL,
                                configMAX_PRIORITIES - 1, NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "failed to spawn mcpwm init task");
        return ESP_FAIL;
    }
    xSemaphoreTake(s_mcpwm_init_done, portMAX_DELAY);
    vSemaphoreDelete(s_mcpwm_init_done);

    gpio_config_t enable_cfg = {};
    enable_cfg.pin_bit_mask = 1ULL << CONFIG_MOTOR_ENABLE_GPIO;
    enable_cfg.mode = GPIO_MODE_OUTPUT;
    enable_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    enable_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    enable_cfg.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&enable_cfg));
    gpio_set_level((gpio_num_t)CONFIG_MOTOR_ENABLE_GPIO, 0);

    write_duties(0.5f, 0.5f, 0.5f);

    BaseType_t ok = xTaskCreatePinnedToCore(motor_task, "motor", 4096, NULL, 5, NULL, 1);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to spawn motor task");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "init ok: A=%d B=%d C=%d SD=%d, carrier=%d Hz, Vbus=%.1f V",
        CONFIG_MOTOR_PWM_A_GPIO, CONFIG_MOTOR_PWM_B_GPIO, CONFIG_MOTOR_PWM_C_GPIO,
        CONFIG_MOTOR_ENABLE_GPIO,
        PWM_RESOLUTION_HZ / PWM_PERIOD_TICKS,
        (double)VBUS_VOLTS);
    return ESP_OK;
}

void motor_enable(void) {
    // Re-zero current sensor bias with the gate still off. The INA240+shunt
    // drifts with temperature, so a fresh zero is needed for stall detection
    // to have accurate readings.
    current_sense_calibrate_bias();
    atomic_store(&g_stalled, false);
    s_stall_count = 0;
    gpio_set_level((gpio_num_t)CONFIG_MOTOR_ENABLE_GPIO, 1);
    atomic_store(&g_motor_enabled, true);
    ESP_LOGI(TAG, "enabled");
}

void motor_disable(void) {
    atomic_store(&g_motor_enabled, false);
    gpio_set_level((gpio_num_t)CONFIG_MOTOR_ENABLE_GPIO, 0);
    ESP_LOGI(TAG, "disabled");
}

void motor_set_velocity(float rad_per_sec) {
    float v = clampf(rad_per_sec, -MAX_VELOCITY, MAX_VELOCITY);
    atomic_store(&g_target_velocity_rad_s, v);
}

void motor_set_voltage_amplitude(float volts) {
    float v = clampf(volts, 0.0f, MAX_VOLTAGE_AMP);
    atomic_store(&g_v_offset_v, v);
}

void motor_set_v_per_rad_s(float v_per_rad_s) {
    if (v_per_rad_s < 0.0f) v_per_rad_s = 0.0f;
    atomic_store(&g_v_per_rad_s, v_per_rad_s);
}

void motor_set_stall_current_a(float amps) {
    // Floor at a sensible minimum; setting it too low (< noise floor) would
    // make the detector trip on idle bias jitter. Zero means "disabled".
    if (amps < 0.0f) amps = 0.0f;
    atomic_store(&g_stall_current_a, amps);
}

void motor_get_status(motor_status_t *out) {
    if (!out) return;
    out->target_velocity_rad_s  = atomic_load(&g_target_velocity_rad_s);
    out->current_velocity_rad_s = atomic_load(&g_current_velocity_rad_s);
    out->voltage_amplitude_v    = atomic_load(&g_voltage_amplitude_v);
    out->v_offset_v             = atomic_load(&g_v_offset_v);
    out->v_per_rad_s            = atomic_load(&g_v_per_rad_s);
    out->i_mag_a                = atomic_load(&g_i_mag_filtered_a);
    out->i_bus_est_a            = atomic_load(&g_i_bus_est_a);
    out->stall_current_a        = atomic_load(&g_stall_current_a);
    out->enabled                = atomic_load(&g_motor_enabled);
    out->stalled                = atomic_load(&g_stalled);
    out->uptime_s               = atomic_load(&g_uptime_s);
}

void motor_set_cal(const motor_cal_t *cal) {
    if (!cal || !cal->valid) return;
    s_cal = *cal;
    // Align voltage solves: V_LL = sqrt(3) * vamp = V_dead + 2*Rs*I_target.
    s_align_vamp = (cal->v_dead + 2.0f * cal->rs_ohm * ALIGN_TARGET_CURRENT_A) / 1.7320508075f;
    ESP_LOGI(TAG, "cal applied: Rs=%.4f Ω, Ls=%.6f H, V_dead=%.4f V, align_vamp=%.4f V",
        (double)cal->rs_ohm, (double)cal->ls_henry, (double)cal->v_dead,
        (double)s_align_vamp);
}

void motor_get_cal(motor_cal_t *out) {
    if (!out) return;
    *out = s_cal;
}

// Drive duties directly at electrical angle 0 (current flows B↔C, A idle).
// Used during identification to apply known DC voltages. Line-to-line BC
// voltage = sqrt(3) * vamp, so amp = vbc / (sqrt(3) * Vbus).
static void ident_apply_voltage_bc(float vbc) {
    float amp = vbc / (1.7320508075f * VBUS_VOLTS);
    if (amp < 0.0f) amp = 0.0f;
    if (amp > 0.30f) amp = 0.30f;   // safety cap inside ID
    float da = 0.5f;
    float db = 0.5f - 0.8660254f * amp;
    float dc = 0.5f + 0.8660254f * amp;
    write_duties(da, db, dc);
}

esp_err_t motor_identify(motor_cal_t *out) {
    if (!out) return ESP_ERR_INVALID_ARG;
    if (atomic_load(&g_motor_enabled)) {
        ESP_LOGE(TAG, "identify refused: motor is enabled");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "identify start");

    // Take the bridge from motor_task and give it a tick to bail.
    atomic_store_explicit(&g_ident_in_progress, true, memory_order_release);
    vTaskDelay(pdMS_TO_TICKS(5));

    // Re-zero the current sensor with the gate still off — the INA240+shunt
    // drifts with temperature, so a fresh zero matches the current thermal
    // state of the board.
    current_sense_calibrate_bias();

    // Enable the gate driver.
    gpio_set_level((gpio_num_t)CONFIG_MOTOR_ENABLE_GPIO, 1);

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
        float ia, ib, ic;
        current_sense_read(&ia, &ib, &ic);
        i_disc = fabsf(ib);
        ESP_LOGI(TAG, "identify Rs discover: V=%.4f, I=%.4f A", (double)v_disc, (double)i_disc);
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
        gpio_set_level((gpio_num_t)CONFIG_MOTOR_ENABLE_GPIO, 0);
        atomic_store_explicit(&g_ident_in_progress, false, memory_order_release);
        ESP_LOGE(TAG, "identify: could not find a usable Rs probe voltage "
                      "(last V=%.4f, I=%.4f) — check wiring and current sensor",
            (double)v_disc, (double)i_disc);
        return ESP_FAIL;
    }

    // Five ascending probes 1.0×..1.4× of v_disc.
    const float scale[5] = { 1.0f, 1.1f, 1.2f, 1.3f, 1.4f };
    float v_pts[5], i_pts[5];
    int n_valid = 0;
    for (int i = 0; i < 5; i++) {
        float v_probe = v_disc * scale[i];
        ident_apply_voltage_bc(v_probe);
        vTaskDelay(pdMS_TO_TICKS(80));
        float ia, ib, ic;
        current_sense_read(&ia, &ib, &ic);
        float i_loop = fabsf(ib);
        ESP_LOGI(TAG, "identify Rs probe: V=%.4f, I=%.4f A", (double)v_probe, (double)i_loop);
        if (i_loop > I_SAT_LIMIT) break;     // past linear range — stop here
        if (i_loop < 0.3f)        continue;  // suspiciously low — skip but keep going
        v_pts[n_valid] = v_probe;
        i_pts[n_valid] = i_loop;
        n_valid++;
    }
    ident_apply_voltage_bc(0.0f);
    vTaskDelay(pdMS_TO_TICKS(30));

    if (n_valid < 3) {
        gpio_set_level((gpio_num_t)CONFIG_MOTOR_ENABLE_GPIO, 0);
        atomic_store_explicit(&g_ident_in_progress, false, memory_order_release);
        ESP_LOGE(TAG, "identify Rs: only %d valid probes — can't fit", n_valid);
        return ESP_FAIL;
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
    if (fabsf(denom) < 1e-9f) {
        gpio_set_level((gpio_num_t)CONFIG_MOTOR_ENABLE_GPIO, 0);
        atomic_store_explicit(&g_ident_in_progress, false, memory_order_release);
        ESP_LOGE(TAG, "identify Rs: degenerate probe data");
        return ESP_FAIL;
    }
    float r_loop = (n_valid * sum_iv - sum_i * sum_v) / denom;
    float v_dead = (sum_v - r_loop * sum_i) / n_valid;
    float rs = r_loop * 0.5f;
    ESP_LOGI(TAG, "identify Rs fit: n=%d, R_loop=%.4f Ω, V_dead=%.4f V",
        n_valid, (double)r_loop, (double)v_dead);

    if (r_loop <= 0.0f) {
        gpio_set_level((gpio_num_t)CONFIG_MOTOR_ENABLE_GPIO, 0);
        atomic_store_explicit(&g_ident_in_progress, false, memory_order_release);
        ESP_LOGE(TAG, "identify Rs: negative slope — wiring or sensor issue");
        return ESP_FAIL;
    }

    // --- Ls measurement ---
    //
    // Bump decimation to 1 (20 kHz sampling) and capture the current rise
    // after a step. τ = first time at which i ≥ 0.632 · i_final.

    float v_step = v_dead + 2.0f * r_loop;   // targets i_final ≈ 2 A through BC pair
    if (v_step < 0.05f) v_step = 0.05f;
    if (v_step > 0.5f)  v_step = 0.5f;

    s_commutation_decimation = 1;
    vTaskDelay(pdMS_TO_TICKS(2));
    ESP_LOGI(TAG, "identify L step: V=%.4f (targeting i_final ~ %.2f A)",
        (double)v_step, (double)((v_step - v_dead) / r_loop));

    const int n_samples = 200;          // 200 × 50 µs = 10 ms capture
    static float samples[200];
    int64_t t0 = esp_timer_get_time();
    ident_apply_voltage_bc(v_step);
    for (int i = 0; i < n_samples; i++) {
        int64_t deadline_us = t0 + (int64_t)i * 50;
        while (esp_timer_get_time() < deadline_us) { /* spin */ }
        float ia, ib, ic;
        current_sense_read(&ia, &ib, &ic);
        samples[i] = fabsf(ib);
    }
    ident_apply_voltage_bc(0.0f);
    vTaskDelay(pdMS_TO_TICKS(20));
    s_commutation_decimation = COMMUTATION_DECIMATION_DEFAULT;

    // i_final = average of last 20 samples.
    float i_final = 0.0f;
    for (int i = n_samples - 20; i < n_samples; i++) i_final += samples[i];
    i_final /= 20.0f;
    if (i_final < 0.01f) {
        gpio_set_level((gpio_num_t)CONFIG_MOTOR_ENABLE_GPIO, 0);
        atomic_store_explicit(&g_ident_in_progress, false, memory_order_release);
        ESP_LOGE(TAG, "identify L: final current %.4f A too low", (double)i_final);
        return ESP_FAIL;
    }
    float i_thresh = 0.6321f * i_final;
    int idx_tau = -1;
    for (int i = 0; i < n_samples; i++) {
        if (samples[i] >= i_thresh) { idx_tau = i; break; }
    }
    if (idx_tau <= 0) {
        gpio_set_level((gpio_num_t)CONFIG_MOTOR_ENABLE_GPIO, 0);
        atomic_store_explicit(&g_ident_in_progress, false, memory_order_release);
        ESP_LOGE(TAG, "identify L: could not extract τ from step response");
        return ESP_FAIL;
    }
    float tau_s = idx_tau * 50e-6f;
    float l_loop = r_loop * tau_s;
    float ls = l_loop * 0.5f;
    ESP_LOGI(TAG, "identify L: i_final=%.4f A, idx_tau=%d, τ=%.3f ms, L_loop=%.6f H",
        (double)i_final, idx_tau, (double)(tau_s * 1000.0f), (double)l_loop);

    // Release the bridge.
    gpio_set_level((gpio_num_t)CONFIG_MOTOR_ENABLE_GPIO, 0);
    write_duties(0.5f, 0.5f, 0.5f);

    motor_cal_t cal = {
        .valid    = true,
        .rs_ohm   = rs,
        .ls_henry = ls,
        .v_dead   = v_dead,
    };
    motor_set_cal(&cal);
    atomic_store_explicit(&g_ident_in_progress, false, memory_order_release);

    *out = cal;
    ESP_LOGI(TAG, "identify ok: Rs=%.4f Ω, Ls=%.6f H, V_dead=%.4f V",
        (double)rs, (double)ls, (double)v_dead);
    return ESP_OK;
}
