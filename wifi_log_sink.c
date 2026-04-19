#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>

#include "esp_log.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "wifi_log_sink.h"

static const char *TAG = "wifi_log";

typedef int (*vprintf_like_t)(const char *fmt, va_list ap);

static vprintf_like_t s_prev_vprintf = NULL;
static int s_listen_sock = -1;
static int s_client_sock = -1;
static SemaphoreHandle_t s_sock_mutex = NULL;
static TaskHandle_t s_server_task = NULL;
static bool s_enabled = false;

static void close_client_socket_locked(void)
{
    if (s_client_sock >= 0) {
        close(s_client_sock);
        s_client_sock = -1;
    }
}

static void close_listen_socket_locked(void)
{
    if (s_listen_sock >= 0) {
        close(s_listen_sock);
        s_listen_sock = -1;
    }
}

static void log_server_task(void *arg)
{
    (void)arg;

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_port = htons(CONFIG_USBIP_WIFI_LOG_PORT),
    };

    int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_fd < 0) {
        ESP_LOGE(TAG, "log socket() failed: %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    (void)setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (bind(listen_fd, (const struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "log bind() failed: %d", errno);
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }
    if (listen(listen_fd, 1) != 0) {
        ESP_LOGE(TAG, "log listen() failed: %d", errno);
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }

    if (s_sock_mutex) {
        xSemaphoreTake(s_sock_mutex, portMAX_DELAY);
        s_listen_sock = listen_fd;
        xSemaphoreGive(s_sock_mutex);
    }

    ESP_LOGI(TAG, "Telnet log server listening on TCP %d", CONFIG_USBIP_WIFI_LOG_PORT);

    while (s_enabled) {
        struct sockaddr_in caddr;
        socklen_t clen = sizeof(caddr);
        int client_fd = accept(listen_fd, (struct sockaddr *)&caddr, &clen);
        if (client_fd < 0) {
            if (s_enabled) {
                ESP_LOGW(TAG, "log accept() failed: %d", errno);
            }
            continue;
        }

        struct timeval snd_to = {
            .tv_sec = 0,
            .tv_usec = 20000,
        };
        (void)setsockopt(client_fd, SOL_SOCKET, SO_SNDTIMEO, &snd_to, sizeof(snd_to));

        xSemaphoreTake(s_sock_mutex, portMAX_DELAY);
        close_client_socket_locked();
        s_client_sock = client_fd;
        xSemaphoreGive(s_sock_mutex);

        ESP_LOGI(TAG, "Telnet log client connected");
    }

    if (s_sock_mutex) {
        xSemaphoreTake(s_sock_mutex, portMAX_DELAY);
        close_client_socket_locked();
        close_listen_socket_locked();
        xSemaphoreGive(s_sock_mutex);
    }

    s_server_task = NULL;
    vTaskDelete(NULL);
}

static int wifi_log_vprintf(const char *fmt, va_list ap)
{
    va_list copy;
    va_copy(copy, ap);

    int ret = 0;
    if (s_prev_vprintf) {
        ret = s_prev_vprintf(fmt, ap);
    }

    if (!s_enabled || !fmt) {
        va_end(copy);
        return ret;
    }

    char msg[320];
    int n = vsnprintf(msg, sizeof(msg), fmt, copy);
    va_end(copy);
    if (n <= 0) {
        return ret;
    }

    size_t len = (size_t)n;
    if (len >= sizeof(msg)) {
        len = sizeof(msg) - 1;
    }

    if (xSemaphoreTake(s_sock_mutex, 0) == pdTRUE) {
        if (s_client_sock >= 0) {
            int sent = send(s_client_sock, msg, len, 0);
            if (sent < 0) {
                close_client_socket_locked();
            }
        }
        xSemaphoreGive(s_sock_mutex);
    }

    return ret;
}

esp_err_t wifi_log_sink_init(void)
{
#if !CONFIG_USBIP_WIFI_LOG_ENABLE
    return ESP_OK;
#else
    if (s_enabled) {
        return ESP_OK;
    }

    s_sock_mutex = xSemaphoreCreateMutex();
    if (!s_sock_mutex) {
        return ESP_ERR_NO_MEM;
    }

    s_prev_vprintf = esp_log_set_vprintf(wifi_log_vprintf);
    s_enabled = true;

    BaseType_t ok = xTaskCreate(log_server_task,
                                "wifi_log_srv",
                                4096,
                                NULL,
                                4,
                                &s_server_task);
    if (ok != pdPASS) {
        s_enabled = false;
        esp_log_set_vprintf(s_prev_vprintf);
        s_prev_vprintf = NULL;
        vSemaphoreDelete(s_sock_mutex);
        s_sock_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Telnet log forwarding enabled on TCP port %d", CONFIG_USBIP_WIFI_LOG_PORT);
    return ESP_OK;
#endif
}

void wifi_log_sink_deinit(void)
{
#if !CONFIG_USBIP_WIFI_LOG_ENABLE
    return;
#else
    if (!s_enabled) {
        return;
    }

    s_enabled = false;
    if (s_prev_vprintf) {
        esp_log_set_vprintf(s_prev_vprintf);
        s_prev_vprintf = NULL;
    }

    if (s_sock_mutex) {
        xSemaphoreTake(s_sock_mutex, portMAX_DELAY);
        close_client_socket_locked();
        close_listen_socket_locked();
        xSemaphoreGive(s_sock_mutex);

        /* Give accept() loop a chance to exit after listen socket is closed. */
        for (int i = 0; i < 20 && s_server_task != NULL; i++) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        vSemaphoreDelete(s_sock_mutex);
        s_sock_mutex = NULL;
    }
#endif
}
