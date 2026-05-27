#include "current_sense.h"

#include <atomic>

#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_log.h"

#include "sdkconfig.h"

using std::atomic_load_explicit;
using std::atomic_store_explicit;
using std::memory_order_relaxed;

static const char *TAG = "isens";

// MKS ESP32 FOC V1.0 typical layout: INA240 phase A -> GPIO 39 (ADC1_CH3),
// phase B -> GPIO 36 (ADC1_CH0). Verify on this board before trusting the
// numbers below.
#define CHAN_A      ADC_CHANNEL_3   // GPIO 39
#define CHAN_B      ADC_CHANNEL_0   // GPIO 36

// INA240A2 gain = 50 V/V. Common shunt = 0.005 Ω. So 1 A across the shunt
// produces 0.25 V at the amp output. Update if your shunt differs.
#define V_PER_AMP   0.25f

// 12 dB attenuation gives a full-scale of ~3.3 V on the ADC pin.
#define ADC_ATTEN   ADC_ATTEN_DB_12
#define ADC_FULL_SCALE_MV 3300
#define ADC_RESOLUTION    4095

// EMA filter coefficient applied to the PWM-synced single-sample current
// estimates. With 1 kHz sample rate from motor_task, alpha=0.1 gives a ~10 ms
// time constant - smooths out residual ripple while staying responsive.
#define EMA_ALPHA   0.1f

static adc_oneshot_unit_handle_t s_adc;   // NULL until current_sense_init()
static int s_bias_a_mv = 1650;
static int s_bias_b_mv = 1650;

static std::atomic<float> s_filtered_a_amps = 0.0f;
static std::atomic<float> s_filtered_b_amps = 0.0f;

static int read_mv_fast(adc_channel_t ch) {
    int raw = 0;
    adc_oneshot_read(s_adc, ch, &raw);
    return (raw * ADC_FULL_SCALE_MV) / ADC_RESOLUTION;
}

static int read_mv_averaged(adc_channel_t ch, int n) {
    int sum = 0;
    for (int i = 0; i < n; i++) {
        int raw = 0;
        adc_oneshot_read(s_adc, ch, &raw);
        sum += raw;
    }
    return ((sum / n) * ADC_FULL_SCALE_MV) / ADC_RESOLUTION;
}

esp_err_t current_sense_init(void) {
    adc_oneshot_unit_init_cfg_t unit_cfg = {};
    unit_cfg.unit_id = ADC_UNIT_1;
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &s_adc), TAG, "adc unit");

    adc_oneshot_chan_cfg_t ch_cfg = {};
    ch_cfg.atten = ADC_ATTEN;
    ch_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc, CHAN_A, &ch_cfg), TAG, "chan a");
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc, CHAN_B, &ch_cfg), TAG, "chan b");

    ESP_LOGI(TAG, "ready: phase A on ADC1_CH3 (GPIO 39), phase B on ADC1_CH0 (GPIO 36)");
    return ESP_OK;
}

esp_err_t current_sense_calibrate_bias(void) {
    // Motor is off when this runs - the INA240 output is steady at the bias
    // point. A long average gives a clean reference.
    s_bias_a_mv = read_mv_averaged(CHAN_A, 256);
    s_bias_b_mv = read_mv_averaged(CHAN_B, 256);
    ESP_LOGI(TAG, "zero-current bias: A=%d mV, B=%d mV (expect ~1650 mV)",
        s_bias_a_mv, s_bias_b_mv);
    return ESP_OK;
}

void current_sense_sample_sync(void) {
    if (!s_adc) return;   // not initialized yet

    int mv_a = read_mv_fast(CHAN_A);
    int mv_b = read_mv_fast(CHAN_B);
    float ia_inst = (mv_a - s_bias_a_mv) / 1000.0f / V_PER_AMP;
    float ib_inst = (mv_b - s_bias_b_mv) / 1000.0f / V_PER_AMP;

    float fa = atomic_load_explicit(&s_filtered_a_amps, memory_order_relaxed);
    float fb = atomic_load_explicit(&s_filtered_b_amps, memory_order_relaxed);
    fa += EMA_ALPHA * (ia_inst - fa);
    fb += EMA_ALPHA * (ib_inst - fb);
    atomic_store_explicit(&s_filtered_a_amps, fa, memory_order_relaxed);
    atomic_store_explicit(&s_filtered_b_amps, fb, memory_order_relaxed);
}

void current_sense_read(float *ia, float *ib, float *ic) {
    float fa = atomic_load_explicit(&s_filtered_a_amps, memory_order_relaxed);
    float fb = atomic_load_explicit(&s_filtered_b_amps, memory_order_relaxed);
    if (ia) *ia = fa;
    if (ib) *ib = fb;
    if (ic) *ic = -(fa + fb);
}

void current_sense_read_raw(int *raw_a, int *raw_b) {
    int ra = 0, rb = 0;
    if (s_adc) {
        adc_oneshot_read(s_adc, CHAN_A, &ra);
        adc_oneshot_read(s_adc, CHAN_B, &rb);
    }
    if (raw_a) *raw_a = ra;
    if (raw_b) *raw_b = rb;
}
