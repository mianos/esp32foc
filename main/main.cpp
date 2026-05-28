#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "MotorWebServer.h"
#include "Settings.h"
#include "WebServer.h"
#include "WifiConnection.h"

#include "current_sense.h"
#include "motor.h"

static const char *TAG = "main";

extern "C" void app_main(void) {
    static CurrentSense current_sense;
    static Motor motor(current_sense);

    ESP_ERROR_CHECK(motor.init());
    ESP_ERROR_CHECK(current_sense.init());
    ESP_ERROR_CHECK(current_sense.calibrate_bias());

    static Settings settings;
    if (settings.load_hostname("").empty()) {
        ESP_LOGI(TAG, "seeding default hostname 'robofoc' into NVS");
        settings.save_hostname("robofoc");
    }

    {
        Motor::Cal cal;
        if (settings.load_cal(cal)) {
            motor.set_cal(cal);
            ESP_LOGI(TAG, "motor cal loaded from NVS: Rs=%.4f Ω, Ls=%.6f H, V_dead=%.4f V",
                static_cast<double>(cal.rs_ohm), static_cast<double>(cal.ls_henry),
                static_cast<double>(cal.v_dead));
        } else {
            ESP_LOGW(TAG, "no motor cal in NVS — POST /calibrate to identify");
        }
    }

    {
        float v_offset, v_per, stall_a;
        settings.load_vhz(v_offset, v_per, stall_a);
        motor.set_voltage_amplitude(v_offset);
        motor.set_v_per_rad_s(v_per);
        motor.set_stall_current_a(stall_a);
        ESP_LOGI(TAG, "tuning loaded: v_offset=%.4f V, v_per_rad_s=%.6f V·s/rad, stall=%.2f A",
            static_cast<double>(v_offset), static_cast<double>(v_per), static_cast<double>(stall_a));

        // Slew has its own key; if unset, the motor keeps the Kconfig default.
        float slew;
        if (settings.load_slew(slew)) {
            motor.set_slew_rad_s2(slew);
            ESP_LOGI(TAG, "slew loaded from NVS: %.1f rad/s^2", static_cast<double>(slew));
        }
    }

    static WifiConnection wifi(settings);

    // No boot alignment pulse — the motor_task ALIGN phase parks the rotor
    // automatically on every enable, using a cal-derived voltage that stays
    // safely under the motor's current rating. Bridge stays disabled until
    // the user POSTs enabled:true.

    wifi.wait_for_ip();

    static WebContext web_ctx(&wifi.manager());
    static MotorWebServer web(&web_ctx, motor, current_sense, settings);
    ESP_ERROR_CHECK(web.start());
    ESP_LOGI(TAG, "webserver up: POST /motor {\"velocity_rad_s\":N,\"voltage_v\":N,"
                  "\"v_per_rad_s\":N,\"stall_current_a\":N,\"slew_rad_s2\":N,\"pole_pairs\":N,\"enabled\":bool}, "
                  "GET /motor for status, POST/GET /calibrate for motor identification, "
                  "POST /pump {\"duty\":0-100}, POST/GET /pump_range "
                  "{\"min_rad_s\":N,\"max_rad_s\":N}");

    // Nothing left to do in app_main. The motor task drives commutation on
    // core 1, the webserver workers handle requests, we just block forever.
    while (true) {
        vTaskDelay(portMAX_DELAY);
    }
}
