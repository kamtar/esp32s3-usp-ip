#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Enable ESP log streaming over a TCP port.
 *
 * Keeps default log output active while also serving logs to one TCP client.
 */
esp_err_t wifi_log_sink_init(void);

/**
 * @brief Disable TCP log streaming and restore default log callback.
 */
void wifi_log_sink_deinit(void);

#ifdef __cplusplus
}
#endif
