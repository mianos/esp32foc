#include "current_sense.h"

#include "esp_check.h"
#include "esp_log.h"

#include "sdkconfig.h"

namespace {

constexpr const char *TAG = "isens";

// MKS ESP32 FOC V1.0 typical layout: INA240 phase A -> GPIO 39 (ADC1_CH3),
// phase B -> GPIO 36 (ADC1_CH0). Verify on this board before trusting the
// numbers below.
constexpr adc_channel_t CHAN_A = ADC_CHANNEL_3;   // GPIO 39
constexpr adc_channel_t CHAN_B = ADC_CHANNEL_0;   // GPIO 36

// INA240A2 gain = 50 V/V. Common shunt = 0.005 Ω. So 1 A across the shunt
// produces 0.25 V at the amp output. Update if your shunt differs.
constexpr float V_PER_AMP = 0.25f;

// 12 dB attenuation gives a full-scale of ~3.3 V on the ADC pin.
constexpr adc_atten_t ADC_ATTEN_SETTING = ADC_ATTEN_DB_12;
constexpr int ADC_FULL_SCALE_MV = 3300;
constexpr int ADC_RESOLUTION    = 4095;

// EMA filter coefficient applied to the PWM-synced single-sample current
// estimates. With 1 kHz sample rate from the motor task, alpha=0.1 gives a
// ~10 ms time constant - smooths out residual ripple while staying responsive.
constexpr float EMA_ALPHA = 0.1f;

}  // namespace

int CurrentSense::read_mv_fast(adc_channel_t ch) const {
    int raw = 0;
    adc_oneshot_read(adc_, ch, &raw);
    return (raw * ADC_FULL_SCALE_MV) / ADC_RESOLUTION;
}

int CurrentSense::read_mv_averaged(adc_channel_t ch, int n) const {
    int sum = 0;
    for (int i = 0; i < n; i++) {
        int raw = 0;
        adc_oneshot_read(adc_, ch, &raw);
        sum += raw;
    }
    return ((sum / n) * ADC_FULL_SCALE_MV) / ADC_RESOLUTION;
}

esp_err_t CurrentSense::init() {
    adc_oneshot_unit_init_cfg_t unit_cfg = {};
    unit_cfg.unit_id = ADC_UNIT_1;
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &adc_), TAG, "adc unit");

    adc_oneshot_chan_cfg_t ch_cfg = {};
    ch_cfg.atten = ADC_ATTEN_SETTING;
    ch_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(adc_, CHAN_A, &ch_cfg), TAG, "chan a");
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(adc_, CHAN_B, &ch_cfg), TAG, "chan b");

    ESP_LOGI(TAG, "ready: phase A on ADC1_CH3 (GPIO 39), phase B on ADC1_CH0 (GPIO 36)");
    return ESP_OK;
}

esp_err_t CurrentSense::calibrate_bias() {
    // Motor is off when this runs - the INA240 output is steady at the bias
    // point. A long average gives a clean reference.
    bias_a_mv_ = read_mv_averaged(CHAN_A, 256);
    bias_b_mv_ = read_mv_averaged(CHAN_B, 256);
    ESP_LOGI(TAG, "zero-current bias: A=%d mV, B=%d mV (expect ~1650 mV)",
        bias_a_mv_, bias_b_mv_);
    return ESP_OK;
}

void CurrentSense::sample_sync() {
    if (!adc_) return;   // not initialized yet

    int mv_a = read_mv_fast(CHAN_A);
    int mv_b = read_mv_fast(CHAN_B);
    float ia_inst = (mv_a - bias_a_mv_) / 1000.0f / V_PER_AMP;
    float ib_inst = (mv_b - bias_b_mv_) / 1000.0f / V_PER_AMP;

    float fa = filtered_a_amps_.load(std::memory_order_relaxed);
    float fb = filtered_b_amps_.load(std::memory_order_relaxed);
    fa += EMA_ALPHA * (ia_inst - fa);
    fb += EMA_ALPHA * (ib_inst - fb);
    filtered_a_amps_.store(fa, std::memory_order_relaxed);
    filtered_b_amps_.store(fb, std::memory_order_relaxed);
}

CurrentSense::Phases CurrentSense::read() const {
    float fa = filtered_a_amps_.load(std::memory_order_relaxed);
    float fb = filtered_b_amps_.load(std::memory_order_relaxed);
    return Phases{fa, fb, -(fa + fb)};
}

CurrentSense::RawAdc CurrentSense::read_raw() const {
    int ra = 0, rb = 0;
    if (adc_) {
        adc_oneshot_read(adc_, CHAN_A, &ra);
        adc_oneshot_read(adc_, CHAN_B, &rb);
    }
    return RawAdc{ra, rb};
}
