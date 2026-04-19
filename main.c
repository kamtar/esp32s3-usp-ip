/*
 * main.c
 *
 * Entry point for the ESP32-S3-Zero USB/IP Host project.
 *
 * Boot sequence:
 *   1. Connect to Wi-Fi
 *   2. Initialise USB Host driver (starts background tasks)
 *   3. Start the USB/IP TCP server on port 3240
 *
 * SPDX-FileCopyrightText: 2026
 * SPDX-License-Identifier: MIT
 */

#include "esp_log.h"
#include "esp_err.h"

#include "wifi_manager.h"
#include "wifi_log_sink.h"
#include "usb_host_driver.h"
#include "usbip_server.h"

static const char *TAG = "main";

void app_main(void)
{
#if CONFIG_USBIP_DEBUG_LOGS
    esp_log_level_set("usbip_server", ESP_LOG_DEBUG);
    esp_log_level_set("usb_host_drv", ESP_LOG_DEBUG);
    esp_log_level_set("wifi_log", ESP_LOG_DEBUG);
#endif

    ESP_LOGI(TAG, "=== ESP32-S3-Zero USB/IP Host ===");

    /* Step 1: Connect to Wi-Fi */
    ESP_LOGI(TAG, "Connecting to Wi-Fi...");
    ESP_ERROR_CHECK(wifi_manager_connect());

    /* Optional: mirror logs over Wi-Fi for headless debugging */
    ESP_ERROR_CHECK(wifi_log_sink_init());

    /* Step 2: Initialise USB Host driver */
    ESP_LOGI(TAG, "Initialising USB Host driver...");
    ESP_ERROR_CHECK(usb_host_driver_init());

    /* Step 3: Start USB/IP server */
    ESP_LOGI(TAG, "Starting USB/IP server on port %d...", CONFIG_USBIP_SERVER_PORT);
    ESP_ERROR_CHECK(usbip_server_start());

    ESP_LOGI(TAG, "Ready.  Connect a USB device to the OTG port,");
    ESP_LOGI(TAG, "then run on your PC:");
    ESP_LOGI(TAG, "  usbip list   -r <ESP32_IP>");
    ESP_LOGI(TAG, "  usbip attach -r <ESP32_IP> -b 1-1");
}
