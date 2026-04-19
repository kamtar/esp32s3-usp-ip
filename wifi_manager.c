/*
 * wifi_manager.c
 *
 * Connects the ESP32-S3 to a Wi-Fi AP using the credentials from Kconfig.
 * Blocks until an IP address is obtained or a 30-second timeout elapses.
 *
 * SPDX-FileCopyrightText: 2026
 * SPDX-License-Identifier: MIT
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "wifi_manager.h"

static const char *TAG = "wifi_mgr";

#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
#define WIFI_MAX_RETRY      10

static EventGroupHandle_t s_wifi_events = NULL;
static int s_retry_count = 0;
static esp_netif_t *s_netif = NULL;

static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_count < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_count++;
            ESP_LOGI(TAG, "retry Wi-Fi (%d/%d)", s_retry_count, WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_events, WIFI_FAIL_BIT);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_count = 0;
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

esp_err_t wifi_manager_connect(void)
{
    s_wifi_events = xEventGroupCreate();
    if (!s_wifi_events) return ESP_ERR_NO_MEM;

    /* Initialise NVS (required by Wi-Fi driver) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "nvs_flash_init");

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                         ESP_EVENT_ANY_ID,
                                                         wifi_event_handler,
                                                         NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                         IP_EVENT_STA_GOT_IP,
                                                         wifi_event_handler,
                                                         NULL, NULL));

    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid              = CONFIG_USBIP_WIFI_SSID,
            .password          = CONFIG_USBIP_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "connecting to SSID: %s", CONFIG_USBIP_WIFI_SSID);

    EventBits_t bits = xEventGroupWaitBits(s_wifi_events,
                                            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE, pdFALSE,
                                            pdMS_TO_TICKS(30000));
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Wi-Fi connected");
        return ESP_OK;
    }
    ESP_LOGE(TAG, "Wi-Fi connection failed");
    return ESP_ERR_TIMEOUT;
}

void wifi_manager_disconnect(void)
{
    esp_wifi_disconnect();
    esp_wifi_stop();
    esp_wifi_deinit();
    if (s_netif) {
        esp_netif_destroy(s_netif);
        s_netif = NULL;
    }
}
