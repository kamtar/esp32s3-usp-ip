#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "usb/usb_host.h"
#include "usb/usb_types_ch9.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum number of endpoints per device (excluding EP0) */
#define USB_HOST_MAX_EP  16

/**
 * @brief Snapshot of an attached USB device's information.
 *
 * The usbip_server reads from this structure; the usb_host_driver writes it.
 * Access must be protected by the mutex in usb_host_driver.
 */
typedef struct {
    bool                        valid;          /**< true when a device is attached */
    usb_device_handle_t         handle;         /**< USB Host Library device handle */
    usb_device_info_t           info;           /**< speed, address, etc.           */
    const usb_config_desc_t    *config_desc;    /**< active configuration descriptor */
    const usb_device_desc_t    *device_desc;    /**< device descriptor               */
    uint8_t                     bus_num;        /**< always 1 on ESP32-S3            */
    uint8_t                     dev_id;         /**< USB device address              */
} usb_device_snapshot_t;

/**
 * @brief Initialise the USB Host driver and start its daemon task.
 *
 * Must be called after the USB/IP server is ready to receive device events.
 *
 * @return ESP_OK on success.
 */
esp_err_t usb_host_driver_init(void);

/**
 * @brief Obtain a pointer to the shared device snapshot.
 *
 * Caller must call usb_host_driver_release_snapshot() when done.
 *
 * @param[out] snap  Filled with the current device snapshot.
 * @return ESP_OK, or ESP_ERR_NOT_FOUND if no device is attached.
 */
esp_err_t usb_host_driver_get_snapshot(usb_device_snapshot_t *snap);

/**
 * @brief Submit a control transfer to the attached device (EP0).
 *
 * Blocks until the transfer completes or times out.
 *
 * @param setup  8-byte USB setup packet.
 * @param data   Data buffer (IN or OUT depending on bmRequestType).
 * @param len    Length of data buffer.
 * @param actual Actual bytes transferred (OUT parameter).
 * @param timeout_ms Timeout in milliseconds.
 * @return ESP_OK on success.
 */
esp_err_t usb_host_driver_ctrl_xfer(const usb_setup_packet_t *setup,
                                     uint8_t *data, uint16_t len,
                                     uint16_t *actual, uint32_t timeout_ms);

/**
 * @brief Submit a non-control endpoint transfer (bulk/interrupt).
 *
 * The endpoint address includes direction in bit7 (e.g. 0x81 for EP1 IN).
 *
 * @param ep_addr Endpoint address including direction.
 * @param data Data buffer for transfer payload.
 * @param len Transfer length in bytes.
 * @param actual Actual bytes transferred (OUT parameter).
 * @param timeout_ms Timeout in milliseconds.
 * @return ESP_OK on success.
 */
esp_err_t usb_host_driver_ep_xfer(uint8_t ep_addr,
                                   uint8_t *data,
                                   uint16_t len,
                                   uint16_t *actual,
                                   uint32_t usbip_transfer_flags,
                                   uint32_t timeout_ms);

/**
 * @brief Submit an isochronous endpoint transfer.
 *
 * @param ep_addr Endpoint address including direction.
 * @param data Transfer payload buffer (packed, without per-packet padding).
 * @param len Total packed transfer length in bytes.
 * @param num_packets Number of isochronous packets.
 * @param pkt_len Expected length for each packet.
 * @param actual Total bytes transferred.
 * @param pkt_actual Actual bytes transferred for each packet.
 * @param pkt_status USB/IP-compatible status per packet (0 on success, negative on error).
 * @param error_count Number of packets that completed with error status.
 * @param timeout_ms Timeout in milliseconds.
 * @return ESP_OK on success.
 */
esp_err_t usb_host_driver_ep_isoc_xfer(uint8_t ep_addr,
                                        uint8_t *data,
                                        uint32_t len,
                                        int32_t num_packets,
                                        const uint32_t *pkt_len,
                                        uint32_t *actual,
                                        uint32_t *pkt_actual,
                                        int32_t *pkt_status,
                                        uint32_t *error_count,
                                        uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
