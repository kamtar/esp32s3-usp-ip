#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the USB/IP TCP server.
 *
 * Spawns a FreeRTOS task that listens on CONFIG_USBIP_SERVER_PORT (default 3240).
 * One client connection is served at a time; the server returns to listening
 * after a client disconnects.
 *
 * @return ESP_OK on success.
 */
esp_err_t usbip_server_start(void);

/**
 * @brief Stop the USB/IP TCP server gracefully.
 */
void usbip_server_stop(void);

#ifdef __cplusplus
}
#endif
