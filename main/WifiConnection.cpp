#include "WifiConnection.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"

namespace {
constexpr const char *TAG = "wifi";
}  // namespace

WifiConnection::WifiConnection(NvsStorageManager &storage)
    : got_ip_(xSemaphoreCreateBinary()),
      wifi_(storage, &WifiConnection::on_got_ip, this) {
    // Second WIFI_EVENT handler (alongside wifimanager's) to surface
    // association failures the wifimanager swallows silently.
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
        &WifiConnection::on_wifi_event, nullptr));
    ESP_LOGI(TAG, "wifi manager started; if unprovisioned, use ESP-Touch v2 app");
}

void WifiConnection::wait_for_ip() {
    ESP_LOGI(TAG, "waiting for wifi to come up before starting webserver");
    xSemaphoreTake(got_ip_, portMAX_DELAY);
}

void WifiConnection::on_got_ip(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base != IP_EVENT || id != IP_EVENT_STA_GOT_IP) return;
    WifiConnection *self = static_cast<WifiConnection *>(arg);
    ip_event_got_ip_t *e = static_cast<ip_event_got_ip_t *>(data);
    const char *hn = nullptr;
    esp_netif_get_hostname(e->esp_netif, &hn);
    ESP_LOGI(TAG, "wifi connected, host=%s IP=" IPSTR,
        hn ? hn : "(null)", IP2STR(&e->ip_info.ip));
    // Disable DTIM power-save: bursty wake/sleep cycles couple as 1 Hz
    // audible ticks. ~80 mA extra is negligible for a 2 A motor.
    esp_wifi_set_ps(WIFI_PS_NONE);
    xSemaphoreGive(self->got_ip_);
}

// Diagnostic: log WIFI_EVENT state changes that the wifimanager swallows
// silently. Disconnect reason codes are the only way to tell why we can't
// associate with a saved AP.
void WifiConnection::on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data) {
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
            ESP_LOGI(TAG, "WIFI_EVENT id=%ld", static_cast<long>(id));
            break;
    }
}
