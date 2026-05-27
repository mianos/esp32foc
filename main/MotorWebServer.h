#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

#include "JsonWrapper.h"
#include "WebServer.h"

class Motor;
class CurrentSense;
class Settings;

// HTTP control surface for the motor: /motor, /calibrate, /pump, /pump_range
// (plus /healthz from the WebServer base). Holds references to the drive,
// current sensor, and settings store; the static httpd handlers recover the
// instance from req->user_ctx. Owns the runtime pump-range and pole-pair
// values, loaded from Settings at construction.
class MotorWebServer : public WebServer {
public:
    MotorWebServer(WebContext *ctx, Motor &motor, CurrentSense &current_sense, Settings &settings);

    esp_err_t start() override;

private:
    static esp_err_t motor_post_handler(httpd_req_t *req);
    static esp_err_t motor_get_handler(httpd_req_t *req);
    static esp_err_t calibrate_post_handler(httpd_req_t *req);
    static esp_err_t calibrate_get_handler(httpd_req_t *req);
    static esp_err_t pump_post_handler(httpd_req_t *req);
    static esp_err_t pump_range_post_handler(httpd_req_t *req);
    static esp_err_t pump_range_get_handler(httpd_req_t *req);

    void  status_to_json(JsonWrapper &json);
    float elec_to_mech_rpm(float elec_rad_s) const;
    float pump_duty_to_velocity(float duty) const;

    // Duty below this (percent) is treated as "off" — gives a small deadband so
    // PID output hovering near zero doesn't chatter the bridge enable.
    static constexpr float kPumpOffDutyPct = 0.5f;

    Motor        &motor_;
    CurrentSense &current_sense_;
    Settings     &settings_;

    // duty→velocity mapping range. Read in the /pump handler, written by
    // /pump_range. Plain floats: aligned 32-bit loads/stores are atomic on the
    // Xtensa core and a stale pairing across a concurrent update is harmless.
    float pump_min_rad_s_;
    float pump_max_rad_s_;

    // Pole-pair count — RPM display only (drive control is electrical).
    int pole_pairs_;
};
