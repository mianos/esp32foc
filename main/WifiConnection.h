#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "esp_event.h"

#include "WifiManager.h"

class NvsStorageManager;

// Station-mode WiFi bring-up layered on the shared WiFiManager: registers the
// got-IP and diagnostic WIFI_EVENT handlers, disables power-save on connect,
// and blocks until the station acquires an IP. Keeps this glue out of main.
class WifiConnection {
public:
    explicit WifiConnection(NvsStorageManager &storage);

    // Block until the station gets an IP (WiFiManager starts connecting from
    // the constructor).
    void wait_for_ip();

    // The wrapped manager, for consumers that need it (e.g. WebContext).
    WiFiManager &manager() { return wifi_; }

private:
    static void on_got_ip(void *arg, esp_event_base_t base, int32_t id, void *data);
    static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data);

    // Declared before wifi_ so the semaphore exists before WiFiManager's
    // constructor can fire on_got_ip.
    SemaphoreHandle_t got_ip_;
    WiFiManager wifi_;
};
