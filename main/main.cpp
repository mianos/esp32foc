#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif_ip_addr.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <string>

#include "JsonWrapper.h"
#include "NvsStorageManager.h"
#include "WebServer.h"
#include "WifiManager.h"

#include "current_sense.h"
#include "motor.h"

static const char *TAG = "main";

#define POLE_PAIRS           7

// V/Hz defaults applied at boot if NVS doesn't already have values stored.
// v_offset is intentionally conservative — small enough that, even without
// any back-EMF (standstill), the resulting winding current stays below the
// motor's rating for typical low-resistance motors. Tune upward via POST
// /motor {"voltage_v":...} if the motor doesn't start; the value persists
// in NVS. v_per_rad_s should track the motor's back-EMF constant Ke.
#define DEFAULT_V_OFFSET_V      0.10f
#define DEFAULT_V_PER_RAD_S     0.001f

// Pump-control mapping: stillerate's PID POSTs a 0–100 % "duty" to /pump.
// duty 0 → motor stopped; duty>0 → enabled and linearly mapped onto
// [pump_min, pump_max] rad/s. The range is settable via POST /pump_range and
// persists in NVS. Min is the lowest speed the V/Hz drive runs smoothly at, so
// even a small positive duty produces a clean spin rather than a stall.
#define DEFAULT_PUMP_MIN_RAD_S  50.0f
#define DEFAULT_PUMP_MAX_RAD_S  1000.0f

// Duty below this (percent) is treated as "off" — gives a small deadband so
// PID output hovering near zero doesn't chatter the bridge enable.
#define PUMP_OFF_DUTY_PCT       0.5f

static inline float elec_to_mech_rpm(float elec_rad_s) {
    return elec_rad_s * 60.0f / (2.0f * 3.14159265f * (float)POLE_PAIRS);
}

static SemaphoreHandle_t s_wifi_got_ip;

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data) {
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = static_cast<ip_event_got_ip_t *>(data);
        const char *hn = nullptr;
        esp_netif_get_hostname(e->esp_netif, &hn);
        ESP_LOGI(TAG, "wifi connected, host=%s IP=" IPSTR,
            hn ? hn : "(null)", IP2STR(&e->ip_info.ip));
        // Disable DTIM power-save: bursty wake/sleep cycles couple as
        // 1 Hz audible ticks. ~80 mA extra is negligible for a 2 A motor.
        esp_wifi_set_ps(WIFI_PS_NONE);
        xSemaphoreGive(s_wifi_got_ip);
    }
}

// Diagnostic: log WIFI_EVENT state changes that the wifimanager swallows
// silently. Disconnect reason codes are the only way to tell why we
// can't associate with a saved AP.
static void wifi_diag_handler(void *arg, esp_event_base_t base,
                              int32_t id, void *data) {
    if (base != WIFI_EVENT) return;
    switch (id) {
        case WIFI_EVENT_STA_CONNECTED: {
            auto *e = static_cast<wifi_event_sta_connected_t *>(data);
            ESP_LOGI(TAG, "WIFI_EVENT_STA_CONNECTED ssid=%.32s ch=%d",
                e->ssid, e->channel);
            break;
        }
        case WIFI_EVENT_STA_DISCONNECTED: {
            auto *e = static_cast<wifi_event_sta_disconnected_t *>(data);
            ESP_LOGW(TAG, "WIFI_EVENT_STA_DISCONNECTED ssid=%.32s reason=%d "
                "(see esp_wifi_types_generic.h for code meaning)",
                e->ssid, e->reason);
            break;
        }
        case WIFI_EVENT_SCAN_DONE:
            ESP_LOGI(TAG, "WIFI_EVENT_SCAN_DONE");
            break;
        default:
            ESP_LOGI(TAG, "WIFI_EVENT id=%ld", (long)id);
            break;
    }
}

static std::string read_request_body(httpd_req_t *req) {
    std::string body;
    body.reserve(req->content_len);
    char buf[256];
    int remaining = req->content_len;
    while (remaining > 0) {
        int got = httpd_req_recv(req, buf, std::min<int>(remaining, (int)sizeof(buf)));
        if (got <= 0) break;
        body.append(buf, got);
        remaining -= got;
    }
    return body;
}

static void motor_status_to_json(JsonWrapper &json) {
    motor_status_t st;
    motor_get_status(&st);
    json.AddItem("velocity_rad_s",         st.target_velocity_rad_s);
    json.AddItem("current_velocity_rad_s", st.current_velocity_rad_s);
    json.AddItem("velocity_rpm_mech",      elec_to_mech_rpm(st.current_velocity_rad_s));
    json.AddItem("voltage_v",              st.voltage_amplitude_v);
    json.AddItem("v_offset_v",             st.v_offset_v);
    json.AddItem("v_per_rad_s",            st.v_per_rad_s);
    json.AddItem("i_mag_a",                st.i_mag_a);
    json.AddItem("i_bus_est_a",            st.i_bus_est_a);
    json.AddItem("stall_current_a",        st.stall_current_a);
    json.AddItem("enabled",                st.enabled);
    json.AddItem("stalled",                st.stalled);
    json.AddItem("uptime_s",               (int)st.uptime_s);

    // Raw phase current snapshot — useful for debugging bias drift.
    float ia, ib, ic;
    current_sense_read(&ia, &ib, &ic);
    json.AddItem("ia_a", ia);
    json.AddItem("ib_a", ib);
    json.AddItem("ic_a", ic);
}

static void cal_to_json(const motor_cal_t &cal, JsonWrapper &json) {
    json.AddItem("valid", cal.valid);
    if (cal.valid) {
        json.AddItem("rs_ohm",   cal.rs_ohm);
        json.AddItem("ls_henry", cal.ls_henry);
        json.AddItem("v_dead",   cal.v_dead);
    }
}

static NvsStorageManager *s_nvs = nullptr;

static bool parse_float(const std::string &s, float &out) {
    char *end = nullptr;
    float v = strtof(s.c_str(), &end);
    if (end == s.c_str()) return false;
    out = v;
    return true;
}

static bool save_cal_to_nvs(const motor_cal_t &cal) {
    if (!s_nvs) return false;
    bool ok = true;
    ok &= s_nvs->store("rs_ohm",   std::to_string(cal.rs_ohm));
    ok &= s_nvs->store("ls_henry", std::to_string(cal.ls_henry));
    ok &= s_nvs->store("v_dead",   std::to_string(cal.v_dead));
    return ok;
}

static bool load_cal_from_nvs(motor_cal_t &cal) {
    if (!s_nvs) return false;
    std::string rs_s, ls_s, vd_s;
    if (!s_nvs->retrieve("rs_ohm",   rs_s) || rs_s.empty()) return false;
    if (!s_nvs->retrieve("ls_henry", ls_s) || ls_s.empty()) return false;
    if (!s_nvs->retrieve("v_dead",   vd_s) || vd_s.empty()) return false;
    if (!parse_float(rs_s, cal.rs_ohm))   return false;
    if (!parse_float(ls_s, cal.ls_henry)) return false;
    if (!parse_float(vd_s, cal.v_dead))   return false;
    cal.valid = true;
    return true;
}

// V/Hz tuning + stall threshold persist separately from cal so the user can
// adjust without re-identifying the motor.
static bool save_vhz_to_nvs(float v_offset, float v_per_rad_s, float stall_a) {
    if (!s_nvs) return false;
    bool ok = true;
    ok &= s_nvs->store("v_offset",   std::to_string(v_offset));
    ok &= s_nvs->store("v_per_rads", std::to_string(v_per_rad_s));
    ok &= s_nvs->store("stall_a",    std::to_string(stall_a));
    return ok;
}

static void load_vhz_from_nvs_or_defaults(float &v_offset, float &v_per_rad_s, float &stall_a) {
    v_offset    = DEFAULT_V_OFFSET_V;
    v_per_rad_s = DEFAULT_V_PER_RAD_S;
    stall_a     = 3.0f;   // mirrors DEFAULT_STALL_CURRENT_A in motor.c
    if (!s_nvs) return;
    std::string s;
    if (s_nvs->retrieve("v_offset",   s) && !s.empty()) parse_float(s, v_offset);
    if (s_nvs->retrieve("v_per_rads", s) && !s.empty()) parse_float(s, v_per_rad_s);
    if (s_nvs->retrieve("stall_a",    s) && !s.empty()) parse_float(s, stall_a);
}

// duty→velocity mapping range. Read in the /pump handler, written by
// /pump_range. Plain floats: aligned 32-bit loads/stores are atomic on the
// Xtensa core and a stale pairing across a concurrent update is harmless.
static float s_pump_min_rad_s = DEFAULT_PUMP_MIN_RAD_S;
static float s_pump_max_rad_s = DEFAULT_PUMP_MAX_RAD_S;

static float pump_duty_to_velocity(float duty) {
    if (duty < 0.0f)   duty = 0.0f;
    if (duty > 100.0f) duty = 100.0f;
    return s_pump_min_rad_s + (duty / 100.0f) * (s_pump_max_rad_s - s_pump_min_rad_s);
}

static bool save_pump_range_to_nvs(float min_v, float max_v) {
    if (!s_nvs) return false;
    bool ok = true;
    ok &= s_nvs->store("pump_min", std::to_string(min_v));
    ok &= s_nvs->store("pump_max", std::to_string(max_v));
    return ok;
}

static void load_pump_range_from_nvs_or_defaults(float &min_v, float &max_v) {
    min_v = DEFAULT_PUMP_MIN_RAD_S;
    max_v = DEFAULT_PUMP_MAX_RAD_S;
    if (!s_nvs) return;
    std::string s;
    if (s_nvs->retrieve("pump_min", s) && !s.empty()) parse_float(s, min_v);
    if (s_nvs->retrieve("pump_max", s) && !s.empty()) parse_float(s, max_v);
}

class MotorWebServer : public WebServer {
public:
    explicit MotorWebServer(WebContext *ctx) : WebServer(ctx) {}

    esp_err_t start() override {
        esp_err_t r = WebServer::start();
        if (r != ESP_OK) return r;

        httpd_uri_t motor_post = {
            .uri = "/motor",
            .method = HTTP_POST,
            .handler = motor_post_handler,
            .user_ctx = this,
        };
        httpd_register_uri_handler(server, &motor_post);

        httpd_uri_t motor_get = {
            .uri = "/motor",
            .method = HTTP_GET,
            .handler = motor_get_handler,
            .user_ctx = this,
        };
        httpd_register_uri_handler(server, &motor_get);

        httpd_uri_t cal_post = {
            .uri = "/calibrate",
            .method = HTTP_POST,
            .handler = calibrate_post_handler,
            .user_ctx = this,
        };
        httpd_register_uri_handler(server, &cal_post);

        httpd_uri_t cal_get = {
            .uri = "/calibrate",
            .method = HTTP_GET,
            .handler = calibrate_get_handler,
            .user_ctx = this,
        };
        httpd_register_uri_handler(server, &cal_get);

        // Compatible with stillerate's RESTMotorController: POST {"duty":0-100}.
        httpd_uri_t pump_post = {
            .uri = "/pump",
            .method = HTTP_POST,
            .handler = pump_post_handler,
            .user_ctx = this,
        };
        httpd_register_uri_handler(server, &pump_post);

        httpd_uri_t pump_range_post = {
            .uri = "/pump_range",
            .method = HTTP_POST,
            .handler = pump_range_post_handler,
            .user_ctx = this,
        };
        httpd_register_uri_handler(server, &pump_range_post);

        httpd_uri_t pump_range_get = {
            .uri = "/pump_range",
            .method = HTTP_GET,
            .handler = pump_range_get_handler,
            .user_ctx = this,
        };
        httpd_register_uri_handler(server, &pump_range_get);

        return ESP_OK;
    }

private:
    static esp_err_t motor_post_handler(httpd_req_t *req) {
        std::string body = read_request_body(req);
        if (body.empty()) {
            return sendJsonError(req, 400, "empty body");
        }

        auto json = JsonWrapper::Parse(body);
        if (json.Empty()) {
            return sendJsonError(req, 400, "invalid JSON");
        }

        float v;
        bool tuning_changed = false;
        if (json.GetField("velocity_rad_s", v)) {
            motor_set_velocity(v);
        }
        if (json.GetField("voltage_v", v)) {
            motor_set_voltage_amplitude(v);
            tuning_changed = true;
        }
        if (json.GetField("v_per_rad_s", v)) {
            motor_set_v_per_rad_s(v);
            tuning_changed = true;
        }
        if (json.GetField("stall_current_a", v)) {
            motor_set_stall_current_a(v);
            tuning_changed = true;
        }
        bool enabled;
        if (json.GetField("enabled", enabled)) {
            if (enabled) motor_enable();
            else         motor_disable();
        }

        // Persist tuning whenever it changes so settings survive a reboot.
        if (tuning_changed) {
            motor_status_t st;
            motor_get_status(&st);
            save_vhz_to_nvs(st.v_offset_v, st.v_per_rad_s, st.stall_current_a);
        }

        JsonWrapper resp;
        motor_status_to_json(resp);
        httpd_resp_set_type(req, "application/json");
        std::string out = resp.ToString();
        httpd_resp_sendstr(req, out.c_str());
        return ESP_OK;
    }

    static esp_err_t motor_get_handler(httpd_req_t *req) {
        JsonWrapper resp;
        motor_status_to_json(resp);
        httpd_resp_set_type(req, "application/json");
        std::string out = resp.ToString();
        httpd_resp_sendstr(req, out.c_str());
        return ESP_OK;
    }

    static esp_err_t calibrate_post_handler(httpd_req_t *req) {
        motor_status_t st;
        motor_get_status(&st);
        if (st.enabled) {
            return sendJsonError(req, 409, "motor must be disabled before /calibrate");
        }

        motor_cal_t cal;
        esp_err_t r = motor_identify(&cal);
        if (r != ESP_OK) {
            return sendJsonError(req, 500, "identification failed — check device log");
        }

        if (!save_cal_to_nvs(cal)) {
            ESP_LOGW(TAG, "cal applied to runtime but NVS save failed");
        }

        JsonWrapper resp;
        cal_to_json(cal, resp);
        httpd_resp_set_type(req, "application/json");
        std::string out = resp.ToString();
        httpd_resp_sendstr(req, out.c_str());
        return ESP_OK;
    }

    static esp_err_t calibrate_get_handler(httpd_req_t *req) {
        motor_cal_t cal;
        motor_get_cal(&cal);
        JsonWrapper resp;
        cal_to_json(cal, resp);
        httpd_resp_set_type(req, "application/json");
        std::string out = resp.ToString();
        httpd_resp_sendstr(req, out.c_str());
        return ESP_OK;
    }

    // POST /pump {"duty":0-100[,"name":"..."]} — speed control compatible with
    // stillerate's RESTMotorController. duty maps onto [pump_min,pump_max] rad/s;
    // duty≈0 stops the motor. enable/disable are only toggled on a state change
    // so a periodic PID post doesn't re-trigger the per-enable alignment ramp.
    static esp_err_t pump_post_handler(httpd_req_t *req) {
        std::string body = read_request_body(req);
        if (body.empty()) {
            return sendJsonError(req, 400, "empty body");
        }

        auto json = JsonWrapper::Parse(body);
        if (json.Empty()) {
            return sendJsonError(req, 400, "invalid JSON");
        }

        float duty;
        if (!json.GetField("duty", duty)) {
            return sendJsonError(req, 400, "missing 'duty'");
        }
        if (duty < 0.0f || duty > 100.0f) {
            return sendJsonError(req, 400, "duty out of range [0,100]");
        }

        std::string name;
        json.GetField("name", name);   // optional, informational

        motor_status_t st;
        motor_get_status(&st);

        float velocity = 0.0f;
        bool enabled;
        if (duty <= PUMP_OFF_DUTY_PCT) {
            if (st.enabled) motor_disable();
            enabled = false;
        } else {
            velocity = pump_duty_to_velocity(duty);
            motor_set_velocity(velocity);
            if (!st.enabled) motor_enable();
            enabled = true;
        }

        JsonWrapper resp;
        resp.AddItem("status",         std::string("success"));
        resp.AddItem("received_duty",  (int)(duty + 0.5f));
        resp.AddItem("velocity_rad_s", velocity);
        resp.AddItem("enabled",        enabled);
        httpd_resp_set_type(req, "application/json");
        std::string out = resp.ToString();
        httpd_resp_sendstr(req, out.c_str());
        return ESP_OK;
    }

    // POST /pump_range {"min_rad_s":N,"max_rad_s":N} — set the duty→velocity
    // mapping. Either field may be omitted to change just one end. Persisted.
    static esp_err_t pump_range_post_handler(httpd_req_t *req) {
        std::string body = read_request_body(req);
        if (body.empty()) {
            return sendJsonError(req, 400, "empty body");
        }

        auto json = JsonWrapper::Parse(body);
        if (json.Empty()) {
            return sendJsonError(req, 400, "invalid JSON");
        }

        float new_min = s_pump_min_rad_s;
        float new_max = s_pump_max_rad_s;
        json.GetField("min_rad_s", new_min);
        json.GetField("max_rad_s", new_max);

        if (new_min < 0.0f) {
            return sendJsonError(req, 400, "min_rad_s must be >= 0");
        }
        if (new_max <= new_min) {
            return sendJsonError(req, 400, "max_rad_s must be > min_rad_s");
        }

        s_pump_min_rad_s = new_min;
        s_pump_max_rad_s = new_max;
        if (!save_pump_range_to_nvs(new_min, new_max)) {
            ESP_LOGW(TAG, "pump range applied to runtime but NVS save failed");
        }

        JsonWrapper resp;
        resp.AddItem("min_rad_s", s_pump_min_rad_s);
        resp.AddItem("max_rad_s", s_pump_max_rad_s);
        httpd_resp_set_type(req, "application/json");
        std::string out = resp.ToString();
        httpd_resp_sendstr(req, out.c_str());
        return ESP_OK;
    }

    static esp_err_t pump_range_get_handler(httpd_req_t *req) {
        JsonWrapper resp;
        resp.AddItem("min_rad_s", s_pump_min_rad_s);
        resp.AddItem("max_rad_s", s_pump_max_rad_s);
        httpd_resp_set_type(req, "application/json");
        std::string out = resp.ToString();
        httpd_resp_sendstr(req, out.c_str());
        return ESP_OK;
    }
};

extern "C" void app_main(void) {
    ESP_ERROR_CHECK(motor_init());
    ESP_ERROR_CHECK(current_sense_init());
    ESP_ERROR_CHECK(current_sense_calibrate_bias());

    s_wifi_got_ip = xSemaphoreCreateBinary();

    static NvsStorageManager nv;
    s_nvs = &nv;
    {
        std::string hn;
        if (!nv.retrieve("hostname", hn) || hn.empty()) {
            ESP_LOGI(TAG, "seeding default hostname 'robofoc' into NVS");
            nv.store("hostname", std::string("robofoc"));
        }
    }

    {
        motor_cal_t cal;
        if (load_cal_from_nvs(cal)) {
            motor_set_cal(&cal);
            ESP_LOGI(TAG, "motor cal loaded from NVS: Rs=%.4f Ω, Ls=%.6f H, V_dead=%.4f V",
                (double)cal.rs_ohm, (double)cal.ls_henry, (double)cal.v_dead);
        } else {
            ESP_LOGW(TAG, "no motor cal in NVS — POST /calibrate to identify");
        }
    }

    {
        float v_offset, v_per, stall_a;
        load_vhz_from_nvs_or_defaults(v_offset, v_per, stall_a);
        motor_set_voltage_amplitude(v_offset);
        motor_set_v_per_rad_s(v_per);
        motor_set_stall_current_a(stall_a);
        ESP_LOGI(TAG, "tuning loaded: v_offset=%.4f V, v_per_rad_s=%.6f V·s/rad, stall=%.2f A",
            (double)v_offset, (double)v_per, (double)stall_a);
    }

    load_pump_range_from_nvs_or_defaults(s_pump_min_rad_s, s_pump_max_rad_s);
    ESP_LOGI(TAG, "pump range loaded: duty 0-100%% -> %.1f..%.1f rad/s",
        (double)s_pump_min_rad_s, (double)s_pump_max_rad_s);

    static WiFiManager wifi(nv, wifi_event_handler, nullptr);
    ESP_LOGI(TAG, "wifi manager started; if unprovisioned, use ESP-Touch v2 app");

    // Add a second WIFI_EVENT handler (in parallel with wifimanager's) to
    // surface association failures the wifimanager doesn't log.
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
        wifi_diag_handler, nullptr));

    // No boot alignment pulse — the motor_task ALIGN phase parks the rotor
    // automatically on every enable, using a cal-derived voltage that stays
    // safely under the motor's current rating. Bridge stays disabled until
    // the user POSTs enabled:true.

    ESP_LOGI(TAG, "waiting for wifi to come up before starting webserver");
    xSemaphoreTake(s_wifi_got_ip, portMAX_DELAY);

    static WebContext web_ctx(&wifi);
    static MotorWebServer web(&web_ctx);
    ESP_ERROR_CHECK(web.start());
    ESP_LOGI(TAG, "webserver up: POST /motor {\"velocity_rad_s\":N,\"voltage_v\":N,"
                  "\"v_per_rad_s\":N,\"stall_current_a\":N,\"enabled\":bool}, "
                  "GET /motor for status, POST/GET /calibrate for motor identification, "
                  "POST /pump {\"duty\":0-100}, POST/GET /pump_range "
                  "{\"min_rad_s\":N,\"max_rad_s\":N}");

    // Nothing left to do in app_main. The motor task drives commutation on
    // core 1, the webserver workers handle requests, we just block forever.
    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}
