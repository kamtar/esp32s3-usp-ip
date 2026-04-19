/*
 * usb_host_driver.c
 *
 * Wraps the ESP-IDF USB Host Library to:
 *  - Install and run the USB Host Library daemon task
 *  - Detect device connect/disconnect
 *  - Open the device and cache its descriptors
 *  - Expose a snapshot for the USB/IP server
 *  - Execute synchronous control (EP0) transfers on request
 *
 * The ESP32-S3 USB-OTG peripheral operates at Full Speed (12 Mbps).
 *
 * SPDX-FileCopyrightText: 2026
 * SPDX-License-Identifier: MIT
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "usb/usb_host.h"
#include "sdkconfig.h"

#include "usb_host_driver.h"

static const char *TAG = "usb_host_drv";

/* -------------------------------------------------------------------------
 * Internal state
 * ---------------------------------------------------------------------- */

#define DEV_CONNECTED_BIT   BIT0
#define DEV_OPENED_BIT      BIT1
#define CTRL_DONE_BIT       BIT2
#define CTRL_ERROR_BIT      BIT3
#define DATA_DONE_BIT       BIT4
#define DATA_ERROR_BIT      BIT5

static EventGroupHandle_t   s_events       = NULL;
static SemaphoreHandle_t    s_snap_mutex   = NULL;
static SemaphoreHandle_t    s_xfer_mutex   = NULL;

/* Shared snapshot — write under s_snap_mutex */
static usb_device_snapshot_t s_snapshot = { .valid = false };

/* USB Host client handle */
static usb_host_client_handle_t s_client_hdl = NULL;

/* Control transfer object (one at a time) */
static usb_transfer_t *s_ctrl_xfer = NULL;

/* usb_host_transfer_alloc() reserves setup packet + payload bytes */
#define CTRL_XFER_DATA_MAX_BYTES CONFIG_USB_HOST_CONTROL_TRANSFER_MAX_SIZE
#define DATA_XFER_MAX_BYTES 16384
#define DATA_ISOC_MAX_PACKETS 128

/* Track claimed interfaces for the currently attached device */
static bool s_intf_claimed[USB_HOST_MAX_EP] = {0};
static uint8_t s_intf_alt[USB_HOST_MAX_EP] = {0};
static bool s_intf_alt_valid[USB_HOST_MAX_EP] = {0};

static volatile usb_transfer_status_t s_last_data_status = USB_TRANSFER_STATUS_ERROR;
static volatile usb_transfer_status_t s_last_ctrl_status = USB_TRANSFER_STATUS_ERROR;

/* ESP32-S3 host controller cannot allocate very large endpoint MPS (e.g. HS 512/1024). */
#define USB_HOST_EP_MPS_SOFT_LIMIT 408U

/* -------------------------------------------------------------------------
 * USB Host library daemon task
 *
 * Must be continuously called from a dedicated task so that the library
 * can process internal events (enumeration, power management, etc.).
 * ---------------------------------------------------------------------- */

static void usb_host_lib_task(void *arg)
{
    ESP_LOGI(TAG, "USB Host library task started");

    while (true) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);

        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_LOGI(TAG, "all clients deregistered");
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "all devices freed");
        }
    }
    vTaskDelete(NULL);
}

/* -------------------------------------------------------------------------
 * Control transfer completion callback
 * ---------------------------------------------------------------------- */

static void ctrl_xfer_cb(usb_transfer_t *xfer)
{
    s_last_ctrl_status = xfer->status;
    if (xfer->status == USB_TRANSFER_STATUS_COMPLETED) {
        xEventGroupSetBits(s_events, CTRL_DONE_BIT);
    } else {
        ESP_LOGW(TAG, "ctrl xfer status: %d", xfer->status);
        xEventGroupSetBits(s_events, CTRL_ERROR_BIT);
    }
}

static void data_xfer_cb(usb_transfer_t *xfer)
{
    s_last_data_status = xfer->status;
    if (xfer->status == USB_TRANSFER_STATUS_COMPLETED) {
        xEventGroupSetBits(s_events, DATA_DONE_BIT);
    } else {
        //ESP_LOGW(TAG, "data xfer status: %d", xfer->status);
        xEventGroupSetBits(s_events, DATA_ERROR_BIT);
    }
}

static void clear_claimed_interfaces(void)
{
    memset(s_intf_claimed, 0, sizeof(s_intf_claimed));
    memset(s_intf_alt, 0, sizeof(s_intf_alt));
    memset(s_intf_alt_valid, 0, sizeof(s_intf_alt_valid));
}

static void recover_control_pipe(usb_device_handle_t dev_hdl)
{
    (void)dev_hdl;
    /*
     * ESP-IDF endpoint halt/flush/clear APIs are for non-zero endpoints.
     * Applying them to EP0 causes invalid-arg errors and can destabilize
     * control-session flow. For EP0, only clear local completion state.
     */
    (void)xEventGroupWaitBits(s_events,
                              CTRL_DONE_BIT | CTRL_ERROR_BIT,
                              pdTRUE, pdFALSE,
                              pdMS_TO_TICKS(20));
}

static bool is_hub_device(const usb_device_desc_t *dev_desc,
                          const usb_config_desc_t *cfg_desc)
{
    if (!dev_desc) {
        return false;
    }

    if (dev_desc->bDeviceClass == USB_CLASS_HUB) {
        return true;
    }

    if (!cfg_desc) {
        return false;
    }

    const uint8_t *p = (const uint8_t *)cfg_desc + cfg_desc->bLength;
    const uint8_t *end = (const uint8_t *)cfg_desc + cfg_desc->wTotalLength;
    while (p + 2 <= end) {
        const uint8_t bLength = p[0];
        const uint8_t bType = p[1];
        if (bLength < 2 || (p + bLength) > end) {
            break;
        }

        if (bType == USB_B_DESCRIPTOR_TYPE_INTERFACE && bLength >= USB_INTF_DESC_SIZE) {
            const usb_intf_desc_t *intf = (const usb_intf_desc_t *)p;
            if (intf->bInterfaceClass == USB_CLASS_HUB) {
                return true;
            }
        }
        p += bLength;
    }

    return false;
}

static esp_err_t find_endpoint_desc(const usb_config_desc_t *cfg,
                                    uint8_t ep_addr,
                                    usb_ep_desc_t *ep_desc_out,
                                    uint8_t *intf_num_out,
                                    uint8_t *alt_out)
{
    if (!cfg || !ep_desc_out || !intf_num_out || !alt_out) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t *p = (const uint8_t *)cfg;
    const uint8_t *end = p + cfg->wTotalLength;
    const usb_intf_desc_t *curr_intf = NULL;
    bool has_fallback = false;
    usb_ep_desc_t fallback_ep = {0};
    uint8_t fallback_intf = 0;
    uint8_t fallback_alt = 0;

    p += cfg->bLength;
    while (p + 2 <= end) {
        const uint8_t bLength = p[0];
        const uint8_t bType = p[1];
        if (bLength < 2 || (p + bLength) > end) {
            return ESP_ERR_INVALID_RESPONSE;
        }

        if (bType == USB_B_DESCRIPTOR_TYPE_INTERFACE && bLength >= USB_INTF_DESC_SIZE) {
            curr_intf = (const usb_intf_desc_t *)p;
        } else if (bType == USB_B_DESCRIPTOR_TYPE_ENDPOINT && bLength >= USB_EP_DESC_SIZE) {
            const usb_ep_desc_t *ep = (const usb_ep_desc_t *)p;
            if (ep->bEndpointAddress == ep_addr && curr_intf) {
                uint8_t intf_num = curr_intf->bInterfaceNumber;
                uint8_t alt = curr_intf->bAlternateSetting;

                if (intf_num < USB_HOST_MAX_EP &&
                    s_intf_alt_valid[intf_num] &&
                    s_intf_alt[intf_num] == alt) {
                    *ep_desc_out = *ep;
                    *intf_num_out = intf_num;
                    *alt_out = alt;
                    return ESP_OK;
                }

                if (!has_fallback) {
                    has_fallback = true;
                    fallback_ep = *ep;
                    fallback_intf = intf_num;
                    fallback_alt = alt;
                }
            }
        }
        p += bLength;
    }

    if (has_fallback) {
        *ep_desc_out = fallback_ep;
        *intf_num_out = fallback_intf;
        *alt_out = fallback_alt;
        return ESP_OK;
    }

    return ESP_ERR_NOT_FOUND;
}

static esp_err_t find_endpoint_desc_for_intf_alt(const usb_config_desc_t *cfg,
                                                 uint8_t ep_addr,
                                                 uint8_t intf_num,
                                                 uint8_t alt,
                                                 usb_ep_desc_t *ep_desc_out)
{
    if (!cfg || !ep_desc_out) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t *p = (const uint8_t *)cfg;
    const uint8_t *end = p + cfg->wTotalLength;
    const usb_intf_desc_t *curr_intf = NULL;

    p += cfg->bLength;
    while (p + 2 <= end) {
        const uint8_t bLength = p[0];
        const uint8_t bType = p[1];
        if (bLength < 2 || (p + bLength) > end) {
            return ESP_ERR_INVALID_RESPONSE;
        }

        if (bType == USB_B_DESCRIPTOR_TYPE_INTERFACE && bLength >= USB_INTF_DESC_SIZE) {
            curr_intf = (const usb_intf_desc_t *)p;
        } else if (bType == USB_B_DESCRIPTOR_TYPE_ENDPOINT && bLength >= USB_EP_DESC_SIZE) {
            const usb_ep_desc_t *ep = (const usb_ep_desc_t *)p;
            if (curr_intf &&
                curr_intf->bInterfaceNumber == intf_num &&
                curr_intf->bAlternateSetting == alt &&
                ep->bEndpointAddress == ep_addr) {
                *ep_desc_out = *ep;
                return ESP_OK;
            }
        }
        p += bLength;
    }

    return ESP_ERR_NOT_FOUND;
}

static esp_err_t try_switch_interface_alt(usb_device_handle_t dev_hdl,
                                          uint8_t intf_num,
                                          uint8_t alt)
{
    esp_err_t ret;

    if (s_intf_claimed[intf_num] && s_intf_alt[intf_num] == alt) {
        return ESP_OK;
    }

    if (s_intf_claimed[intf_num]) {
        ret = usb_host_interface_release(s_client_hdl, dev_hdl, intf_num);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "release intf %u failed: %s", intf_num, esp_err_to_name(ret));
            return ret;
        }
        s_intf_claimed[intf_num] = false;
    }

    ret = usb_host_interface_claim(s_client_hdl, dev_hdl, intf_num, alt);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "claim intf %u alt %u failed: %s", intf_num, alt, esp_err_to_name(ret));
        return ret;
    }

    s_intf_claimed[intf_num] = true;
    s_intf_alt[intf_num] = alt;
    s_intf_alt_valid[intf_num] = true;
    return ESP_OK;
}

static esp_err_t ensure_interface_claimed(usb_device_handle_t dev_hdl,
                                          const usb_config_desc_t *cfg,
                                          uint8_t ep_addr,
                                          uint8_t intf_num,
                                          uint8_t alt,
                                          uint8_t *claimed_alt_out)
{
    if (intf_num >= USB_HOST_MAX_EP) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = try_switch_interface_alt(dev_hdl, intf_num, alt);
    if (ret == ESP_OK) {
        if (claimed_alt_out) {
            *claimed_alt_out = alt;
        }
        return ESP_OK;
    }
    if (ret != ESP_ERR_NOT_SUPPORTED || !cfg) {
        return ret;
    }

    const uint8_t *p = (const uint8_t *)cfg;
    const uint8_t *end = p + cfg->wTotalLength;
    const usb_intf_desc_t *curr_intf = NULL;

    p += cfg->bLength;
    while (p + 2 <= end) {
        const uint8_t bLength = p[0];
        const uint8_t bType = p[1];
        if (bLength < 2 || (p + bLength) > end) {
            return ESP_ERR_INVALID_RESPONSE;
        }

        if (bType == USB_B_DESCRIPTOR_TYPE_INTERFACE && bLength >= USB_INTF_DESC_SIZE) {
            curr_intf = (const usb_intf_desc_t *)p;
        } else if (bType == USB_B_DESCRIPTOR_TYPE_ENDPOINT && bLength >= USB_EP_DESC_SIZE) {
            const usb_ep_desc_t *ep = (const usb_ep_desc_t *)p;
            if (!curr_intf || curr_intf->bInterfaceNumber != intf_num) {
                p += bLength;
                continue;
            }

            uint8_t cand_alt = curr_intf->bAlternateSetting;
            if (cand_alt == alt || ep->bEndpointAddress != ep_addr) {
                p += bLength;
                continue;
            }

            esp_err_t cand_ret = try_switch_interface_alt(dev_hdl, intf_num, cand_alt);
            if (cand_ret == ESP_OK) {
                ESP_LOGW(TAG,
                         "intf %u alt %u unsupported for ep 0x%02X, remapped to alt %u",
                         intf_num,
                         alt,
                         ep_addr,
                         cand_alt);
                if (claimed_alt_out) {
                    *claimed_alt_out = cand_alt;
                }
                return ESP_OK;
            }
        }

        p += bLength;
    }

    return ret;
}

static esp_err_t map_data_xfer_status(usb_transfer_status_t st)
{
    switch (st) {
        case USB_TRANSFER_STATUS_COMPLETED:
            return ESP_OK;
        case USB_TRANSFER_STATUS_TIMED_OUT:
            return ESP_ERR_TIMEOUT;
        case USB_TRANSFER_STATUS_NO_DEVICE:
            return ESP_ERR_NOT_FOUND;
        case USB_TRANSFER_STATUS_STALL:
            return ESP_ERR_INVALID_RESPONSE;
        default:
            return ESP_FAIL;
    }
}

static int32_t map_transfer_status_to_usbip_status(usb_transfer_status_t st)
{
    switch (st) {
        case USB_TRANSFER_STATUS_COMPLETED:
            return 0;
        case USB_TRANSFER_STATUS_TIMED_OUT:
            return -110; /* ETIMEDOUT */
        case USB_TRANSFER_STATUS_CANCELED:
            return -104; /* ECONNRESET */
        case USB_TRANSFER_STATUS_STALL:
            return -32;  /* EPIPE */
        case USB_TRANSFER_STATUS_NO_DEVICE:
            return -19;  /* ENODEV */
        case USB_TRANSFER_STATUS_OVERFLOW:
            return -75;  /* EOVERFLOW */
        default:
            return -1;
    }
}

/* -------------------------------------------------------------------------
 * Client event callback — called from the client task context
 * ---------------------------------------------------------------------- */

static void client_event_cb(const usb_host_client_event_msg_t *msg, void *arg)
{
    switch (msg->event) {
        case USB_HOST_CLIENT_EVENT_NEW_DEV: {
            uint8_t dev_addr = msg->new_dev.address;
            ESP_LOGI(TAG, "new USB device on address %d", dev_addr);

            usb_device_handle_t dev_hdl;
            esp_err_t ret = usb_host_device_open(s_client_hdl, dev_addr, &dev_hdl);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "failed to open device: %s", esp_err_to_name(ret));
                break;
            }

            /* Fetch descriptors */
            const usb_device_desc_t   *dev_desc = NULL;
            const usb_config_desc_t   *cfg_desc = NULL;
            usb_device_info_t          dev_info;

            usb_host_get_device_descriptor(dev_hdl, &dev_desc);
            usb_host_get_active_config_descriptor(dev_hdl, &cfg_desc);
            usb_host_device_info(dev_hdl, &dev_info);

            if (!dev_desc) {
                ESP_LOGW(TAG, "device descriptor unavailable on address %d", dev_addr);
                usb_host_device_close(s_client_hdl, dev_hdl);
                break;
            }

            if (is_hub_device(dev_desc, cfg_desc)) {
                ESP_LOGI(TAG, "ignoring USB hub at address %d", dev_addr);
                usb_host_device_close(s_client_hdl, dev_hdl);
                break;
            }

            xSemaphoreTake(s_snap_mutex, portMAX_DELAY);
            bool already_selected = s_snapshot.valid;
            xSemaphoreGive(s_snap_mutex);
            if (already_selected) {
                ESP_LOGW(TAG,
                         "additional USB device %04X:%04X on addr %d ignored (single export mode)",
                         dev_desc->idVendor, dev_desc->idProduct, dev_addr);
                usb_host_device_close(s_client_hdl, dev_hdl);
                break;
            }

            ESP_LOGI(TAG, "VID:PID = %04X:%04X  class=%02X sub=%02X proto=%02X",
                     dev_desc->idVendor, dev_desc->idProduct,
                     dev_desc->bDeviceClass, dev_desc->bDeviceSubClass,
                     dev_desc->bDeviceProtocol);
            ESP_LOGI(TAG, "device speed enum=%d address=%d",
                     (int)dev_info.speed,
                     (int)dev_addr);

            /* Update snapshot */
            xSemaphoreTake(s_snap_mutex, portMAX_DELAY);
            s_snapshot.valid       = true;
            s_snapshot.handle      = dev_hdl;
            s_snapshot.info        = dev_info;
            s_snapshot.device_desc = dev_desc;
            s_snapshot.config_desc = cfg_desc;
            s_snapshot.bus_num     = 1;
            s_snapshot.dev_id      = dev_addr;
            xSemaphoreGive(s_snap_mutex);

            clear_claimed_interfaces();

            xEventGroupSetBits(s_events, DEV_CONNECTED_BIT | DEV_OPENED_BIT);
            break;
        }

        case USB_HOST_CLIENT_EVENT_DEV_GONE: {
            ESP_LOGI(TAG, "USB device disconnected");

            bool exported_gone = false;
            xSemaphoreTake(s_snap_mutex, portMAX_DELAY);
            if (s_snapshot.valid && msg->dev_gone.dev_hdl == s_snapshot.handle) {
                for (uint8_t i = 0; i < USB_HOST_MAX_EP; i++) {
                    if (s_intf_claimed[i]) {
                        usb_host_interface_release(s_client_hdl, s_snapshot.handle, i);
                    }
                }
                usb_host_device_close(s_client_hdl, s_snapshot.handle);
                memset(&s_snapshot, 0, sizeof(s_snapshot));
                s_snapshot.valid = false;
                exported_gone = true;
            } else {
                ESP_LOGI(TAG, "disconnected device was not the exported one");
            }
            xSemaphoreGive(s_snap_mutex);

            if (exported_gone) {
                clear_claimed_interfaces();
                xEventGroupClearBits(s_events, DEV_CONNECTED_BIT | DEV_OPENED_BIT);
            }
            break;
        }

        default:
            break;
    }
}

/* -------------------------------------------------------------------------
 * Client task — runs the USB Host client event loop
 * ---------------------------------------------------------------------- */

static void usb_host_client_task(void *arg)
{
    ESP_LOGI(TAG, "USB Host client task started");

    usb_host_client_config_t client_cfg = {
        .is_synchronous    = false,
        .max_num_event_msg = 5,
        .async = {
            .client_event_callback = client_event_cb,
            .callback_arg          = NULL,
        },
    };
    ESP_ERROR_CHECK(usb_host_client_register(&client_cfg, &s_client_hdl));

    /* Allocate a reusable control transfer (max 512 bytes data) */
    ESP_ERROR_CHECK(usb_host_transfer_alloc(CTRL_XFER_DATA_MAX_BYTES + sizeof(usb_setup_packet_t),
                                             0, &s_ctrl_xfer));
    s_ctrl_xfer->callback = ctrl_xfer_cb;

    while (true) {
        usb_host_client_handle_events(s_client_hdl, portMAX_DELAY);
    }

    /* Cleanup (not reached in normal operation) */
    usb_host_transfer_free(s_ctrl_xfer);
    usb_host_client_deregister(s_client_hdl);
    vTaskDelete(NULL);
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

esp_err_t usb_host_driver_init(void)
{
    s_events     = xEventGroupCreate();
    s_snap_mutex = xSemaphoreCreateMutex();
    s_xfer_mutex = xSemaphoreCreateMutex();
    if (!s_events || !s_snap_mutex || !s_xfer_mutex) {
        return ESP_ERR_NO_MEM;
    }

    /* Install the USB Host Library */
    usb_host_config_t host_cfg = {
        .skip_phy_setup      = false,
        .intr_flags          = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_RETURN_ON_ERROR(usb_host_install(&host_cfg),
                        TAG, "usb_host_install failed");

    /* Start the library daemon task (handles internal lib events) */
    xTaskCreate(usb_host_lib_task, "usb_lib", 4096, NULL, 10, NULL);

    /* Start the client task (handles device connect/disconnect) */
    xTaskCreate(usb_host_client_task, "usb_client", 4096, NULL, 9, NULL);

    ESP_LOGI(TAG, "USB Host driver initialised — waiting for device...");
    return ESP_OK;
}

esp_err_t usb_host_driver_get_snapshot(usb_device_snapshot_t *snap)
{
    xSemaphoreTake(s_snap_mutex, portMAX_DELAY);
    *snap = s_snapshot;
    xSemaphoreGive(s_snap_mutex);
    return snap->valid ? ESP_OK : ESP_ERR_NOT_FOUND;
}

esp_err_t usb_host_driver_ctrl_xfer(const usb_setup_packet_t *setup,
                                     uint8_t *data, uint16_t len,
                                     uint16_t *actual, uint32_t timeout_ms)
{
    if (!setup || !actual) {
        return ESP_ERR_INVALID_ARG;
    }
    if (len > CTRL_XFER_DATA_MAX_BYTES) {
        ESP_LOGW(TAG, "ctrl xfer too large: %u", len);
        return ESP_ERR_INVALID_SIZE;
    }

    xSemaphoreTake(s_snap_mutex, portMAX_DELAY);
    bool valid = s_snapshot.valid;
    usb_device_handle_t dev_hdl = s_snapshot.handle;
    xSemaphoreGive(s_snap_mutex);

    if (!valid) {
        return ESP_ERR_NOT_FOUND;
    }

    xSemaphoreTake(s_xfer_mutex, portMAX_DELAY);

    /* Set up the transfer */
    memcpy(s_ctrl_xfer->data_buffer, setup, sizeof(usb_setup_packet_t));
    if (data && len > 0) {
        /* For OUT transfers copy data after setup; for IN the buffer is filled
         * by the host controller. */
        if (!(setup->bmRequestType & 0x80)) {
            memcpy(s_ctrl_xfer->data_buffer + sizeof(usb_setup_packet_t),
                   data, len);
        }
    }
    s_ctrl_xfer->num_bytes    = sizeof(usb_setup_packet_t) + len;
    s_ctrl_xfer->device_handle = dev_hdl;
    s_ctrl_xfer->bEndpointAddress = 0; /* EP0 */

    /* Clear completion bits before submitting */
    xEventGroupClearBits(s_events, CTRL_DONE_BIT | CTRL_ERROR_BIT);
    s_last_ctrl_status = USB_TRANSFER_STATUS_ERROR;

    esp_err_t ret = usb_host_transfer_submit_control(s_client_hdl, s_ctrl_xfer);
    if (ret == ESP_ERR_NOT_FINISHED) {
        recover_control_pipe(dev_hdl);
        ret = usb_host_transfer_submit_control(s_client_hdl, s_ctrl_xfer);
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "submit_control failed: %s", esp_err_to_name(ret));
        xSemaphoreGive(s_xfer_mutex);
        return ret;
    }

    /* Wait for completion */
    EventBits_t bits = xEventGroupWaitBits(s_events,
                                            CTRL_DONE_BIT | CTRL_ERROR_BIT,
                                            pdTRUE, pdFALSE,
                                            pdMS_TO_TICKS(timeout_ms));
    if (bits & CTRL_DONE_BIT) {
        int32_t transferred = s_ctrl_xfer->actual_num_bytes -
                              (int32_t)sizeof(usb_setup_packet_t);
        if (transferred < 0) transferred = 0;
        *actual = (uint16_t)transferred;

        /* Copy IN data back to caller's buffer */
        if ((setup->bmRequestType & 0x80) && data && *actual > 0) {
            memcpy(data,
                   s_ctrl_xfer->data_buffer + sizeof(usb_setup_packet_t),
                   *actual);
        }
        xSemaphoreGive(s_xfer_mutex);
        return ESP_OK;
    } else if (bits & CTRL_ERROR_BIT) {
        *actual = 0;
        recover_control_pipe(dev_hdl);
        xSemaphoreGive(s_xfer_mutex);
        return ESP_ERR_INVALID_RESPONSE;
    } else {
        *actual = 0;
        ESP_LOGW(TAG, "control transfer timed out");
        recover_control_pipe(dev_hdl);
        xSemaphoreGive(s_xfer_mutex);
        return ESP_ERR_TIMEOUT;
    }
}

esp_err_t usb_host_driver_ep_xfer(uint8_t ep_addr,
                                   uint8_t *data,
                                   uint16_t len,
                                   uint16_t *actual,
                                   uint32_t usbip_transfer_flags,
                                   uint32_t timeout_ms)
{
    if (!actual) {
        return ESP_ERR_INVALID_ARG;
    }
    if (len > DATA_XFER_MAX_BYTES) {
        return ESP_ERR_INVALID_SIZE;
    }

    xSemaphoreTake(s_snap_mutex, portMAX_DELAY);
    bool valid = s_snapshot.valid;
    usb_device_handle_t dev_hdl = s_snapshot.handle;
    const usb_config_desc_t *cfg = s_snapshot.config_desc;
    xSemaphoreGive(s_snap_mutex);

    if (!valid || !cfg) {
        return ESP_ERR_NOT_FOUND;
    }

    usb_ep_desc_t ep_desc;
    uint8_t intf_num = 0;
    uint8_t alt = 0;
    esp_err_t ret = find_endpoint_desc(cfg, ep_addr, &ep_desc, &intf_num, &alt);
    if (ret != ESP_OK) {
        return ret;
    }

    usb_transfer_type_t xfer_type = USB_EP_DESC_GET_XFERTYPE(&ep_desc);
    if (xfer_type == USB_TRANSFER_TYPE_ISOCHRONOUS) {
        /* TODO: requires packet descriptor handling and per-URB framing */
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (xfer_type != USB_TRANSFER_TYPE_BULK && xfer_type != USB_TRANSFER_TYPE_INTR) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t claimed_alt = alt;
    ret = ensure_interface_claimed(dev_hdl, cfg, ep_addr, intf_num, alt, &claimed_alt);
    if (ret != ESP_OK) {
        return ret;
    }

    if (claimed_alt != alt) {
        esp_err_t desc_ret = find_endpoint_desc_for_intf_alt(cfg,
                                                             ep_addr,
                                                             intf_num,
                                                             claimed_alt,
                                                             &ep_desc);
        if (desc_ret == ESP_OK) {
            alt = claimed_alt;
        }
    }

    uint16_t mps = USB_EP_DESC_GET_MPS(&ep_desc);
    if (mps > USB_HOST_EP_MPS_SOFT_LIMIT) {
        ESP_LOGW(TAG,
                 "ep 0x%02X MPS=%u exceeds controller limit=%u (likely HS-only interface)",
                 ep_addr,
                 (unsigned)mps,
                 (unsigned)USB_HOST_EP_MPS_SOFT_LIMIT);
        return ESP_ERR_NOT_SUPPORTED;
    }
    uint32_t dir_in = (ep_addr & USB_B_ENDPOINT_ADDRESS_EP_DIR_MASK) ? 1U : 0U;
    uint16_t submit_len = len;
    if (dir_in && mps > 0 && len > 0 && (len % mps) != 0) {
        submit_len = (uint16_t)(((len + mps - 1U) / mps) * mps);
        if (submit_len > DATA_XFER_MAX_BYTES) {
            return ESP_ERR_INVALID_SIZE;
        }
    }

    xSemaphoreTake(s_xfer_mutex, portMAX_DELAY);

    usb_transfer_t *xfer = NULL;
    ret = usb_host_transfer_alloc(submit_len > 0 ? submit_len : 1, 0, &xfer);
    if (ret != ESP_OK) {
        xSemaphoreGive(s_xfer_mutex);
        return ret;
    }

    xfer->device_handle = dev_hdl;
    xfer->bEndpointAddress = ep_addr;
    xfer->callback = data_xfer_cb;
    xfer->num_bytes = submit_len;
    xfer->timeout_ms = timeout_ms;
    xfer->flags = 0;

    /* USBIP_URB_ZERO_PACKET: append ZLP for bulk/interrupt OUT when required */
    if (!dir_in && (usbip_transfer_flags & 0x00000040U)) {
        xfer->flags |= USB_TRANSFER_FLAG_ZERO_PACK;
    }

    if (!dir_in && data && len > 0) {
        memcpy(xfer->data_buffer, data, len);
    }

    s_last_data_status = USB_TRANSFER_STATUS_ERROR;
    xEventGroupClearBits(s_events, DATA_DONE_BIT | DATA_ERROR_BIT);

    ret = usb_host_transfer_submit(xfer);
    if (ret != ESP_OK) {
        usb_host_transfer_free(xfer);
        xSemaphoreGive(s_xfer_mutex);
        return ret;
    }

    EventBits_t bits = xEventGroupWaitBits(s_events,
                                           DATA_DONE_BIT | DATA_ERROR_BIT,
                                           pdTRUE, pdFALSE,
                                           pdMS_TO_TICKS(timeout_ms));
    if ((bits & (DATA_DONE_BIT | DATA_ERROR_BIT)) == 0) {
        /*
         * timeout_ms on async transfers is not enforced by the host stack.
         * Cancel timed-out transfer explicitly before freeing to avoid freeing
         * a transfer still in-flight.
         */
        esp_err_t h = usb_host_endpoint_halt(dev_hdl, ep_addr);
        esp_err_t f = usb_host_endpoint_flush(dev_hdl, ep_addr);
        esp_err_t c = usb_host_endpoint_clear(dev_hdl, ep_addr);
        if (h != ESP_OK || f != ESP_OK || c != ESP_OK) {
            ESP_LOGW(TAG, "timeout cleanup ep=0x%02x halt=%s flush=%s clear=%s",
                     ep_addr,
                     esp_err_to_name(h),
                     esp_err_to_name(f),
                     esp_err_to_name(c));
        }

        (void)xEventGroupWaitBits(s_events,
                                  DATA_DONE_BIT | DATA_ERROR_BIT,
                                  pdTRUE, pdFALSE,
                                  pdMS_TO_TICKS(20));

        usb_host_transfer_free(xfer);
        xSemaphoreGive(s_xfer_mutex);
        *actual = 0;
        return dir_in ? ESP_OK : ESP_ERR_TIMEOUT;
    }

    int32_t xfer_actual = xfer->actual_num_bytes;
    if (xfer_actual < 0) {
        xfer_actual = 0;
    }
    *actual = (uint16_t)xfer_actual;

    if (dir_in && data && *actual > 0) {
        uint16_t copy_len = (*actual > len) ? len : *actual;
        memcpy(data, xfer->data_buffer, copy_len);
        *actual = copy_len;
    }

    esp_err_t result = map_data_xfer_status(s_last_data_status);
    usb_host_transfer_free(xfer);
    xSemaphoreGive(s_xfer_mutex);
    return result;
}

esp_err_t usb_host_driver_ep_isoc_xfer(uint8_t ep_addr,
                                        uint8_t *data,
                                        uint32_t len,
                                        int32_t num_packets,
                                        const uint32_t *pkt_len,
                                        uint32_t *actual,
                                        uint32_t *pkt_actual,
                                        int32_t *pkt_status,
                                        uint32_t *error_count,
                                        uint32_t timeout_ms)
{
    if (!actual || !pkt_len || !pkt_actual || !pkt_status || !error_count) {
        return ESP_ERR_INVALID_ARG;
    }
    if (num_packets <= 0 || num_packets > DATA_ISOC_MAX_PACKETS) {
        return ESP_ERR_INVALID_ARG;
    }
    if (len > DATA_XFER_MAX_BYTES) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint32_t total_expected = 0;
    for (int32_t i = 0; i < num_packets; i++) {
        total_expected += pkt_len[i];
    }
    if (total_expected != len) {
        return ESP_ERR_INVALID_SIZE;
    }

    xSemaphoreTake(s_snap_mutex, portMAX_DELAY);
    bool valid = s_snapshot.valid;
    usb_device_handle_t dev_hdl = s_snapshot.handle;
    const usb_config_desc_t *cfg = s_snapshot.config_desc;
    xSemaphoreGive(s_snap_mutex);

    if (!valid || !cfg) {
        return ESP_ERR_NOT_FOUND;
    }

    usb_ep_desc_t ep_desc;
    uint8_t intf_num = 0;
    uint8_t alt = 0;
    esp_err_t ret = find_endpoint_desc(cfg, ep_addr, &ep_desc, &intf_num, &alt);
    if (ret != ESP_OK) {
        return ret;
    }

    usb_transfer_type_t xfer_type = USB_EP_DESC_GET_XFERTYPE(&ep_desc);
    if (xfer_type != USB_TRANSFER_TYPE_ISOCHRONOUS) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t claimed_alt = alt;
    ret = ensure_interface_claimed(dev_hdl, cfg, ep_addr, intf_num, alt, &claimed_alt);
    if (ret != ESP_OK) {
        return ret;
    }

    if (claimed_alt != alt) {
        esp_err_t desc_ret = find_endpoint_desc_for_intf_alt(cfg,
                                                             ep_addr,
                                                             intf_num,
                                                             claimed_alt,
                                                             &ep_desc);
        if (desc_ret != ESP_OK) {
            return desc_ret;
        }
        alt = claimed_alt;
    }

    xSemaphoreTake(s_xfer_mutex, portMAX_DELAY);

    usb_transfer_t *xfer = NULL;
    ret = usb_host_transfer_alloc(len > 0 ? len : 1, num_packets, &xfer);
    if (ret != ESP_OK) {
        xSemaphoreGive(s_xfer_mutex);
        return ret;
    }

    xfer->device_handle = dev_hdl;
    xfer->bEndpointAddress = ep_addr;
    xfer->callback = data_xfer_cb;
    xfer->num_bytes = (int)len;
    xfer->timeout_ms = timeout_ms;
    xfer->flags = 0;

    for (int32_t i = 0; i < num_packets; i++) {
        xfer->isoc_packet_desc[i].num_bytes = (int)pkt_len[i];
        xfer->isoc_packet_desc[i].actual_num_bytes = 0;
        xfer->isoc_packet_desc[i].status = USB_TRANSFER_STATUS_COMPLETED;
    }

    bool dir_in = ((ep_addr & USB_B_ENDPOINT_ADDRESS_EP_DIR_MASK) != 0);
    if (!dir_in && data && len > 0) {
        memcpy(xfer->data_buffer, data, len);
    }

    s_last_data_status = USB_TRANSFER_STATUS_ERROR;
    xEventGroupClearBits(s_events, DATA_DONE_BIT | DATA_ERROR_BIT);

    ret = usb_host_transfer_submit(xfer);
    if (ret != ESP_OK) {
        usb_host_transfer_free(xfer);
        xSemaphoreGive(s_xfer_mutex);
        return ret;
    }

    EventBits_t bits = xEventGroupWaitBits(s_events,
                                           DATA_DONE_BIT | DATA_ERROR_BIT,
                                           pdTRUE, pdFALSE,
                                           pdMS_TO_TICKS(timeout_ms));
    if ((bits & (DATA_DONE_BIT | DATA_ERROR_BIT)) == 0) {
        usb_host_endpoint_halt(dev_hdl, ep_addr);
        usb_host_endpoint_flush(dev_hdl, ep_addr);
        usb_host_endpoint_clear(dev_hdl, ep_addr);
        usb_host_transfer_free(xfer);
        xSemaphoreGive(s_xfer_mutex);
        *actual = 0;
        *error_count = (uint32_t)num_packets;
        for (int32_t i = 0; i < num_packets; i++) {
            pkt_actual[i] = 0;
            pkt_status[i] = -110;
        }
        return ESP_ERR_TIMEOUT;
    }

    int32_t xfer_actual = xfer->actual_num_bytes;
    if (xfer_actual < 0) {
        xfer_actual = 0;
    }
    *actual = (uint32_t)xfer_actual;

    if (dir_in && data && *actual > 0) {
        memcpy(data, xfer->data_buffer, *actual);
    }

    uint32_t err = 0;
    for (int32_t i = 0; i < num_packets; i++) {
        int32_t one_actual = xfer->isoc_packet_desc[i].actual_num_bytes;
        if (one_actual < 0) {
            one_actual = 0;
        }
        pkt_actual[i] = (uint32_t)one_actual;

        usb_transfer_status_t one_st = xfer->isoc_packet_desc[i].status;
        pkt_status[i] = map_transfer_status_to_usbip_status(one_st);
        if (one_st != USB_TRANSFER_STATUS_COMPLETED) {
            err++;
        }
    }
    *error_count = err;

    esp_err_t result = map_data_xfer_status(s_last_data_status);
    usb_host_transfer_free(xfer);
    xSemaphoreGive(s_xfer_mutex);
    return result;
}
