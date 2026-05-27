#include "MotorWebServer.h"

#include <algorithm>
#include <numbers>
#include <optional>
#include <string>

#include "esp_log.h"

#include "Settings.h"
#include "current_sense.h"
#include "motor.h"

namespace {

constexpr const char *TAG = "motorweb";

std::string read_request_body(httpd_req_t *req) {
    std::string body;
    body.reserve(req->content_len);
    char buf[256];
    int remaining = req->content_len;
    while (remaining > 0) {
        int got = httpd_req_recv(req, buf, std::min<int>(remaining, static_cast<int>(sizeof(buf))));
        if (got <= 0) break;
        body.append(buf, got);
        remaining -= got;
    }
    return body;
}

void cal_to_json(const Motor::Cal &cal, JsonWrapper &json) {
    json.AddItem("valid", cal.valid);
    if (cal.valid) {
        json.AddItem("rs_ohm",   cal.rs_ohm);
        json.AddItem("ls_henry", cal.ls_henry);
        json.AddItem("v_dead",   cal.v_dead);
    }
}

}  // namespace

MotorWebServer::MotorWebServer(WebContext *ctx, Motor &motor, CurrentSense &current_sense,
                               Settings &settings)
    : WebServer(ctx), motor_(motor), current_sense_(current_sense), settings_(settings) {
    settings_.load_pump_range(pump_min_rad_s_, pump_max_rad_s_);
    ESP_LOGI(TAG, "pump range loaded: duty 0-100%% -> %.1f..%.1f rad/s",
        static_cast<double>(pump_min_rad_s_), static_cast<double>(pump_max_rad_s_));
    pole_pairs_ = settings_.load_pole_pairs();
    ESP_LOGI(TAG, "pole pairs loaded: %d (RPM display only)", pole_pairs_);
}

float MotorWebServer::elec_to_mech_rpm(float elec_rad_s) const {
    return elec_rad_s * 60.0f / (2.0f * std::numbers::pi_v<float> * static_cast<float>(pole_pairs_));
}

float MotorWebServer::pump_duty_to_velocity(float duty) const {
    if (duty < 0.0f)   duty = 0.0f;
    if (duty > 100.0f) duty = 100.0f;
    return pump_min_rad_s_ + (duty / 100.0f) * (pump_max_rad_s_ - pump_min_rad_s_);
}

void MotorWebServer::status_to_json(JsonWrapper &json) {
    Motor::Status st = motor_.status();
    json.AddItem("velocity_rad_s",         st.target_velocity_rad_s);
    json.AddItem("current_velocity_rad_s", st.current_velocity_rad_s);
    json.AddItem("velocity_rpm_mech",      elec_to_mech_rpm(st.current_velocity_rad_s));
    json.AddItem("pole_pairs",             pole_pairs_);
    json.AddItem("voltage_v",              st.voltage_amplitude_v);
    json.AddItem("v_offset_v",             st.v_offset_v);
    json.AddItem("v_per_rad_s",            st.v_per_rad_s);
    json.AddItem("i_mag_a",                st.i_mag_a);
    json.AddItem("i_bus_est_a",            st.i_bus_est_a);
    json.AddItem("stall_current_a",        st.stall_current_a);
    json.AddItem("enabled",                st.enabled);
    json.AddItem("stalled",                st.stalled);
    json.AddItem("uptime_s",               static_cast<int>(st.uptime_s));

    // Raw phase current snapshot — useful for debugging bias drift.
    CurrentSense::Phases ph = current_sense_.read();
    json.AddItem("ia_a", ph.ia);
    json.AddItem("ib_a", ph.ib);
    json.AddItem("ic_a", ph.ic);
}

esp_err_t MotorWebServer::start() {
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

esp_err_t MotorWebServer::motor_post_handler(httpd_req_t *req) {
    MotorWebServer *self = static_cast<MotorWebServer *>(req->user_ctx);
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
        self->motor_.set_velocity(v);
    }
    if (json.GetField("voltage_v", v)) {
        self->motor_.set_voltage_amplitude(v);
        tuning_changed = true;
    }
    if (json.GetField("v_per_rad_s", v)) {
        self->motor_.set_v_per_rad_s(v);
        tuning_changed = true;
    }
    if (json.GetField("stall_current_a", v)) {
        self->motor_.set_stall_current_a(v);
        tuning_changed = true;
    }
    // pole_pairs only scales the RPM readout; persist it on its own NVS key
    // (independent of the V/Hz tuning group) so it survives a reboot.
    int pp;
    if (json.GetField("pole_pairs", pp)) {
        if (pp < 1) pp = 1;
        self->pole_pairs_ = pp;
        self->settings_.save_pole_pairs(pp);
    }
    bool enabled;
    if (json.GetField("enabled", enabled)) {
        if (enabled) self->motor_.enable();
        else         self->motor_.disable();
    }

    // Persist tuning whenever it changes so settings survive a reboot.
    if (tuning_changed) {
        Motor::Status st = self->motor_.status();
        self->settings_.save_vhz(st.v_offset_v, st.v_per_rad_s, st.stall_current_a);
    }

    JsonWrapper resp;
    self->status_to_json(resp);
    httpd_resp_set_type(req, "application/json");
    std::string out = resp.ToString();
    httpd_resp_sendstr(req, out.c_str());
    return ESP_OK;
}

esp_err_t MotorWebServer::motor_get_handler(httpd_req_t *req) {
    MotorWebServer *self = static_cast<MotorWebServer *>(req->user_ctx);
    JsonWrapper resp;
    self->status_to_json(resp);
    httpd_resp_set_type(req, "application/json");
    std::string out = resp.ToString();
    httpd_resp_sendstr(req, out.c_str());
    return ESP_OK;
}

esp_err_t MotorWebServer::calibrate_post_handler(httpd_req_t *req) {
    MotorWebServer *self = static_cast<MotorWebServer *>(req->user_ctx);
    Motor::Status st = self->motor_.status();
    if (st.enabled) {
        return sendJsonError(req, 409, "motor must be disabled before /calibrate");
    }

    std::optional<Motor::Cal> cal_opt = self->motor_.identify();
    if (!cal_opt) {
        return sendJsonError(req, 500, "identification failed — check device log");
    }
    const Motor::Cal &cal = *cal_opt;

    if (!self->settings_.save_cal(cal)) {
        ESP_LOGW(TAG, "cal applied to runtime but NVS save failed");
    }

    JsonWrapper resp;
    cal_to_json(cal, resp);
    httpd_resp_set_type(req, "application/json");
    std::string out = resp.ToString();
    httpd_resp_sendstr(req, out.c_str());
    return ESP_OK;
}

esp_err_t MotorWebServer::calibrate_get_handler(httpd_req_t *req) {
    MotorWebServer *self = static_cast<MotorWebServer *>(req->user_ctx);
    Motor::Cal cal = self->motor_.cal();
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
esp_err_t MotorWebServer::pump_post_handler(httpd_req_t *req) {
    MotorWebServer *self = static_cast<MotorWebServer *>(req->user_ctx);
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

    Motor::Status st = self->motor_.status();

    float velocity = 0.0f;
    bool enabled;
    if (duty <= kPumpOffDutyPct) {
        if (st.enabled) self->motor_.disable();
        enabled = false;
    } else {
        velocity = self->pump_duty_to_velocity(duty);
        self->motor_.set_velocity(velocity);
        if (!st.enabled) self->motor_.enable();
        enabled = true;
    }

    JsonWrapper resp;
    resp.AddItem("status",         std::string("success"));
    resp.AddItem("received_duty",  static_cast<int>(duty + 0.5f));
    resp.AddItem("velocity_rad_s", velocity);
    resp.AddItem("enabled",        enabled);
    httpd_resp_set_type(req, "application/json");
    std::string out = resp.ToString();
    httpd_resp_sendstr(req, out.c_str());
    return ESP_OK;
}

// POST /pump_range {"min_rad_s":N,"max_rad_s":N} — set the duty→velocity
// mapping. Either field may be omitted to change just one end. Persisted.
esp_err_t MotorWebServer::pump_range_post_handler(httpd_req_t *req) {
    MotorWebServer *self = static_cast<MotorWebServer *>(req->user_ctx);
    std::string body = read_request_body(req);
    if (body.empty()) {
        return sendJsonError(req, 400, "empty body");
    }

    auto json = JsonWrapper::Parse(body);
    if (json.Empty()) {
        return sendJsonError(req, 400, "invalid JSON");
    }

    float new_min = self->pump_min_rad_s_;
    float new_max = self->pump_max_rad_s_;
    json.GetField("min_rad_s", new_min);
    json.GetField("max_rad_s", new_max);

    if (new_min < 0.0f) {
        return sendJsonError(req, 400, "min_rad_s must be >= 0");
    }
    if (new_max <= new_min) {
        return sendJsonError(req, 400, "max_rad_s must be > min_rad_s");
    }

    self->pump_min_rad_s_ = new_min;
    self->pump_max_rad_s_ = new_max;
    if (!self->settings_.save_pump_range(new_min, new_max)) {
        ESP_LOGW(TAG, "pump range applied to runtime but NVS save failed");
    }

    JsonWrapper resp;
    resp.AddItem("min_rad_s", self->pump_min_rad_s_);
    resp.AddItem("max_rad_s", self->pump_max_rad_s_);
    httpd_resp_set_type(req, "application/json");
    std::string out = resp.ToString();
    httpd_resp_sendstr(req, out.c_str());
    return ESP_OK;
}

esp_err_t MotorWebServer::pump_range_get_handler(httpd_req_t *req) {
    MotorWebServer *self = static_cast<MotorWebServer *>(req->user_ctx);
    JsonWrapper resp;
    resp.AddItem("min_rad_s", self->pump_min_rad_s_);
    resp.AddItem("max_rad_s", self->pump_max_rad_s_);
    httpd_resp_set_type(req, "application/json");
    std::string out = resp.ToString();
    httpd_resp_sendstr(req, out.c_str());
    return ESP_OK;
}
