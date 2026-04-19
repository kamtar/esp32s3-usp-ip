#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Connect to the configured Wi-Fi network and wait for an IP address.
 *
 * Blocks the calling task until the connection is established or 30 s elapses.
 *
 * @return ESP_OK on success, ESP_ERR_TIMEOUT on failure.
 */
esp_err_t wifi_manager_connect(void);

/**
 * @brief Disconnect from Wi-Fi and release resources.
 */
void wifi_manager_disconnect(void);

#ifdef __cplusplus
}
#endif
