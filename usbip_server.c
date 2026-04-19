/*
 * usbip_server.c
 *
 * Implements the server side of the USB/IP protocol (RFC-style, as documented
 * at https://www.kernel.org/doc/html/latest/usb/usbip_protocol.html).
 *
 * Only the subset needed for a single Full-Speed device on one virtual bus is
 * implemented.  The ESP32-S3 USB-OTG is Full-Speed (12 Mbps) only.
 *
 * Protocol flow:
 *   1. Client opens TCP connection to port 3240.
 *   2. OP_REQ_DEVLIST  → OP_REP_DEVLIST   (device enumeration)
 *   3. OP_REQ_IMPORT   → OP_REP_IMPORT    (attach device)
 *   4. USBIP_CMD_SUBMIT / USBIP_RET_SUBMIT (URB submit/return loop)
 *   5. USBIP_CMD_UNLINK / USBIP_RET_UNLINK (optional URB cancellation)
 *
 * SPDX-FileCopyrightText: 2026
 * SPDX-License-Identifier: MIT
 */

#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "lwip/sockets.h"

#include "sdkconfig.h"
#include "usbip_server.h"
#include "usb_host_driver.h"

static const char *TAG = "usbip_server";

/* -------------------------------------------------------------------------
 * USB/IP protocol constants (network byte order sent via htonl/htons)
 * ---------------------------------------------------------------------- */

/* Version */
#define USBIP_VERSION           0x0111

/* Operation codes */
#define OP_REQUEST              0x80
#define OP_REPLY                0x00
#define OP_DEVLIST              0x05
#define OP_IMPORT               0x03

#define OP_REQ_DEVLIST          ((OP_REQUEST << 8) | OP_DEVLIST)
#define OP_REP_DEVLIST          ((OP_REPLY   << 8) | OP_DEVLIST)
#define OP_REQ_IMPORT           ((OP_REQUEST << 8) | OP_IMPORT)
#define OP_REP_IMPORT           ((OP_REPLY   << 8) | OP_IMPORT)

/* Command codes (used after device is attached) */
#define USBIP_CMD_SUBMIT        0x00000001
#define USBIP_CMD_UNLINK        0x00000002
#define USBIP_RET_SUBMIT        0x00000003
#define USBIP_RET_UNLINK        0x00000004

/* Direction */
#define USBIP_DIR_OUT           0
#define USBIP_DIR_IN            1

/* Status */
#define USBIP_ST_OK             0
#define USBIP_ST_ERROR          (-1)

/* Safety cap for URB payloads accepted by this implementation */
#define USBIP_MAX_TRANSFER_BUFFER   16384
#define USBIP_MAX_ISO_PACKETS       128

/* Virtual bus/device IDs used in responses */
#define USBIP_BUS_ID_STR        "1-1"
#define USBIP_BUS_NUM           1
#define USBIP_DEV_NUM           1

/* -------------------------------------------------------------------------
 * On-wire structures  (all fields big-endian)
 * ---------------------------------------------------------------------- */

/* Common header for OP requests/replies */
typedef struct __attribute__((packed)) {
    uint16_t version;
    uint16_t code;
    uint32_t status;
} op_common_t;

/* OP_REP_DEVLIST device entry */
typedef struct __attribute__((packed)) {
    char     path[256];      /* sysfs path        */
    char     busid[32];      /* "1-1"             */
    uint32_t busnum;
    uint32_t devnum;
    uint32_t speed;          /* 2 = Full Speed    */
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t  bDeviceClass;
    uint8_t  bDeviceSubClass;
    uint8_t  bDeviceProtocol;
    uint8_t  bConfigurationValue;
    uint8_t  bNumConfigurations;
    uint8_t  bNumInterfaces;
} op_devlist_device_t;

/* OP_REP_IMPORT response */
typedef struct __attribute__((packed)) {
    char     path[256];
    char     busid[32];
    uint32_t busnum;
    uint32_t devnum;
    uint32_t speed;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t  bDeviceClass;
    uint8_t  bDeviceSubClass;
    uint8_t  bDeviceProtocol;
    uint8_t  bConfigurationValue;
    uint8_t  bNumConfigurations;
    uint8_t  bNumInterfaces;
} op_import_reply_device_t;

/* USBIP common header (20 bytes) used for CMD_SUBMIT and RET_SUBMIT */
typedef struct __attribute__((packed)) {
    uint32_t command;
    uint32_t seqnum;
    uint32_t devid;     /* busnum<<16 | devnum for CMD; 0 for RET */
    uint32_t direction;
    uint32_t ep;
} usbip_header_basic_t;

/* CMD_SUBMIT additional fields */
typedef struct __attribute__((packed)) {
    uint32_t transfer_flags;
    int32_t  transfer_buffer_length;
    int32_t  start_frame;
    int32_t  number_of_packets;
    int32_t  interval;
    uint8_t  setup[8];
} usbip_cmd_submit_t;

/* RET_SUBMIT additional fields */
typedef struct __attribute__((packed)) {
    int32_t  status;
    int32_t  actual_length;
    int32_t  start_frame;
    int32_t  number_of_packets;
    int32_t  error_count;
    uint8_t  setup[8];   /* zero-padded */
} usbip_ret_submit_t;

/* CMD_UNLINK additional fields */
typedef struct __attribute__((packed)) {
    uint32_t seqnum_unlink;
    uint8_t  padding[24];
} usbip_cmd_unlink_t;

typedef struct __attribute__((packed)) {
    uint32_t offset;
    uint32_t length;
    uint32_t actual_length;
    uint32_t status;
} usbip_iso_packet_desc_t;

/* RET_UNLINK additional fields */
typedef struct __attribute__((packed)) {
    int32_t  status;
    uint8_t  padding[24];
} usbip_ret_unlink_t;

/* -------------------------------------------------------------------------
 * Helper: reliable send / receive
 * ---------------------------------------------------------------------- */

static esp_err_t sock_send_all(int fd, const void *buf, size_t len)
{
    const uint8_t *p = buf;
    while (len > 0) {
        ssize_t n = send(fd, p, len, 0);
        if (n <= 0) {
            ESP_LOGE(TAG, "send error: %d", errno);
            return ESP_FAIL;
        }
        p   += n;
        len -= n;
    }
    return ESP_OK;
}

static esp_err_t sock_recv_all(int fd, void *buf, size_t len)
{
    uint8_t *p = buf;
    while (len > 0) {
        ssize_t n = recv(fd, p, len, 0);
        if (n <= 0) {
            if (n == 0) {
                ESP_LOGI(TAG, "client disconnected");
            } else if (errno == EWOULDBLOCK || errno == EAGAIN) {
                return ESP_ERR_TIMEOUT;
            } else {
                ESP_LOGE(TAG, "recv error: %d", errno);
            }
            return ESP_FAIL;
        }
        p   += n;
        len -= n;
    }
    return ESP_OK;
}

static int32_t esp_err_to_usbip_status(esp_err_t err)
{
    switch (err) {
        case ESP_OK:
            return 0;
        case ESP_ERR_TIMEOUT:
            return -110; /* ETIMEDOUT */
        case ESP_ERR_NOT_FOUND:
            return -19;  /* ENODEV */
        case ESP_ERR_NOT_SUPPORTED:
            return -95;  /* EOPNOTSUPP */
        case ESP_ERR_INVALID_SIZE:
            return -90;  /* EMSGSIZE */
        case ESP_ERR_INVALID_RESPONSE:
            return -32;  /* EPIPE (typical STALL mapping) */
        case ESP_ERR_NO_MEM:
            return -12;  /* ENOMEM */
        default:
            return -1;
    }
}

/* -------------------------------------------------------------------------
 * Fill a device entry from the current snapshot
 * ---------------------------------------------------------------------- */

static void fill_device_entry(op_devlist_device_t *d,
                               const usb_device_snapshot_t *snap)
{
    memset(d, 0, sizeof(*d));
    snprintf(d->path,  sizeof(d->path),  "/sys/devices/platform/vhci/usb1/1-1");
    snprintf(d->busid, sizeof(d->busid), USBIP_BUS_ID_STR);
    d->busnum            = htonl(USBIP_BUS_NUM);
    d->devnum            = htonl(USBIP_DEV_NUM);
    d->speed             = htonl(2); /* USB_SPEED_FULL */
    d->idVendor          = htons(snap->device_desc->idVendor);
    d->idProduct         = htons(snap->device_desc->idProduct);
    d->bcdDevice         = htons(snap->device_desc->bcdDevice);
    d->bDeviceClass      = snap->device_desc->bDeviceClass;
    d->bDeviceSubClass   = snap->device_desc->bDeviceSubClass;
    d->bDeviceProtocol   = snap->device_desc->bDeviceProtocol;
    d->bConfigurationValue = 1;
    d->bNumConfigurations  = snap->device_desc->bNumConfigurations;
    /* Count interfaces from config descriptor */
    d->bNumInterfaces = snap->config_desc ? snap->config_desc->bNumInterfaces : 0;
}

/* -------------------------------------------------------------------------
 * Handle OP_REQ_DEVLIST
 * ---------------------------------------------------------------------- */

static esp_err_t handle_devlist(int client_fd)
{
    usb_device_snapshot_t snap;
    esp_err_t ret = usb_host_driver_get_snapshot(&snap);

    op_common_t reply = {
        .version = htons(USBIP_VERSION),
        .code    = htons(OP_REP_DEVLIST),
        .status  = htonl(0),
    };
    ESP_RETURN_ON_ERROR(sock_send_all(client_fd, &reply, sizeof(reply)),
                        TAG, "send devlist header");

    if (ret != ESP_OK || !snap.valid) {
        /* No device — send count=0 */
        uint32_t count = htonl(0);
        return sock_send_all(client_fd, &count, sizeof(count));
    }

    uint32_t count = htonl(1);
    ESP_RETURN_ON_ERROR(sock_send_all(client_fd, &count, sizeof(count)),
                        TAG, "send device count");

    op_devlist_device_t dev;
    fill_device_entry(&dev, &snap);
    ESP_RETURN_ON_ERROR(sock_send_all(client_fd, &dev, sizeof(dev)),
                        TAG, "send device entry");

    /* Send one interface descriptor stub per interface */
    uint8_t iface_count = snap.config_desc ? snap.config_desc->bNumInterfaces : 0;
    for (uint8_t i = 0; i < iface_count; i++) {
        uint8_t iface_stub[4] = {0, 0, 0, 0}; /* class, subclass, proto, padding */
        /* Try to get interface info from config descriptor */
        if (snap.config_desc) {
            /* Walk config descriptor to find interface descriptors */
            const uint8_t *p = (const uint8_t *)snap.config_desc + snap.config_desc->bLength;
            const uint8_t *end = (const uint8_t *)snap.config_desc + snap.config_desc->wTotalLength;
            uint8_t found = 0;
            while (p < end) {
                if (p[1] == USB_B_DESCRIPTOR_TYPE_INTERFACE && p[2] == i) {
                    iface_stub[0] = p[5]; /* bInterfaceClass    */
                    iface_stub[1] = p[6]; /* bInterfaceSubClass */
                    iface_stub[2] = p[7]; /* bInterfaceProtocol */
                    found = 1;
                    break;
                }
                p += p[0]; /* bLength */
            }
            if (!found) {
                ESP_LOGW(TAG, "interface %d not found in config desc", i);
            }
        }
        ESP_RETURN_ON_ERROR(sock_send_all(client_fd, iface_stub, sizeof(iface_stub)),
                            TAG, "send iface stub %d", i);
    }
    return ESP_OK;
}

/* -------------------------------------------------------------------------
 * Handle OP_REQ_IMPORT
 * ---------------------------------------------------------------------- */

static esp_err_t handle_import(int client_fd, bool *attached)
{
    char busid[32];
    if (sock_recv_all(client_fd, busid, sizeof(busid)) != ESP_OK) {
        return ESP_FAIL;
    }
    busid[sizeof(busid) - 1] = '\0';
    ESP_LOGI(TAG, "client requests import of busid: %s", busid);

    usb_device_snapshot_t snap;
    esp_err_t ret = usb_host_driver_get_snapshot(&snap);
    bool device_ok = (ret == ESP_OK && snap.valid &&
                      strcmp(busid, USBIP_BUS_ID_STR) == 0);

    op_common_t reply = {
        .version = htons(USBIP_VERSION),
        .code    = htons(OP_REP_IMPORT),
        .status  = htonl(device_ok ? 0 : 1),
    };
    ESP_RETURN_ON_ERROR(sock_send_all(client_fd, &reply, sizeof(reply)),
                        TAG, "send import reply header");

    if (!device_ok) {
        ESP_LOGW(TAG, "import failed: no device on busid %s", busid);
        return ESP_FAIL;
    }

    op_import_reply_device_t dev;
    memset(&dev, 0, sizeof(dev));
    snprintf(dev.path,  sizeof(dev.path),  "/sys/devices/platform/vhci/usb1/1-1");
    snprintf(dev.busid, sizeof(dev.busid), USBIP_BUS_ID_STR);
    dev.busnum            = htonl(USBIP_BUS_NUM);
    dev.devnum            = htonl(USBIP_DEV_NUM);
    dev.speed             = htonl(2);
    dev.idVendor          = htons(snap.device_desc->idVendor);
    dev.idProduct         = htons(snap.device_desc->idProduct);
    dev.bcdDevice         = htons(snap.device_desc->bcdDevice);
    dev.bDeviceClass      = snap.device_desc->bDeviceClass;
    dev.bDeviceSubClass   = snap.device_desc->bDeviceSubClass;
    dev.bDeviceProtocol   = snap.device_desc->bDeviceProtocol;
    dev.bConfigurationValue = 1;
    dev.bNumConfigurations  = snap.device_desc->bNumConfigurations;
    dev.bNumInterfaces      = snap.config_desc ? snap.config_desc->bNumInterfaces : 0;

    ESP_RETURN_ON_ERROR(sock_send_all(client_fd, &dev, sizeof(dev)),
                        TAG, "send import device info");

    *attached = true;
    ESP_LOGI(TAG, "device imported successfully");
    return ESP_OK;
}

/* -------------------------------------------------------------------------
 * Handle USBIP_CMD_SUBMIT — the main URB processing loop
 * ---------------------------------------------------------------------- */

static esp_err_t handle_cmd_submit(int client_fd,
                                    const usbip_header_basic_t *hdr)
{
    usbip_cmd_submit_t cmd;
    if (sock_recv_all(client_fd, &cmd, sizeof(cmd)) != ESP_OK) {
        return ESP_FAIL;
    }

    uint32_t dir    = ntohl(hdr->direction);
    uint32_t ep_num = ntohl(hdr->ep);
    int32_t  buf_len = ntohl((uint32_t)cmd.transfer_buffer_length);
    uint32_t seqnum  = ntohl(hdr->seqnum);
    int32_t  num_packets = ntohl((uint32_t)cmd.number_of_packets);
    int32_t  interval = ntohl((uint32_t)cmd.interval);
    uint32_t transfer_flags = ntohl(cmd.transfer_flags);
    bool is_isoc = (num_packets >= 0);

    if (buf_len < 0 || buf_len > USBIP_MAX_TRANSFER_BUFFER) {
        ESP_LOGW(TAG, "invalid transfer length: %"PRId32, buf_len);
        return ESP_FAIL;
    }
    if (!is_isoc && num_packets != -1) {
        ESP_LOGW(TAG, "invalid number_of_packets: %"PRId32, num_packets);
        return ESP_FAIL;
    }
    if (is_isoc && num_packets > USBIP_MAX_ISO_PACKETS) {
        ESP_LOGW(TAG, "too many isoc packets: %"PRId32, num_packets);
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "CMD_SUBMIT seq=%"PRIu32" ep=%"PRIu32" dir=%"PRIu32" len=%"PRId32" flags=0x%08"PRIx32,
             seqnum, ep_num, dir, buf_len, transfer_flags);

    /*
     * Keep the USB/IP session responsive for stream-oriented devices (e.g. CDC/UART)
     * by polling IN endpoints with short transfer time slices.
     */
    uint32_t xfer_timeout_ms = 2000;
    if (ep_num != 0 && !is_isoc) {
        /* Keep streaming endpoints responsive in both directions */
        if (dir == USBIP_DIR_IN) {
            if (interval > 0 && interval < 50) {
                xfer_timeout_ms = (uint32_t)interval;
            } else {
                xfer_timeout_ms = 50;
            }
        } else {
            xfer_timeout_ms = 500;
        }
    }
    if (ep_num != 0) {
        ESP_LOGD(TAG, "seq=%"PRIu32" ep=%"PRIu32" using timeout=%"PRIu32"ms", seqnum, ep_num, xfer_timeout_ms);
    }

    /* Allocate data buffer */
    uint8_t *data_buf = NULL;
    if (buf_len > 0) {
        data_buf = malloc(buf_len);
        if (!data_buf) {
            ESP_LOGE(TAG, "OOM for transfer buffer %"PRId32, buf_len);
            return ESP_ERR_NO_MEM;
        }
    }

    int32_t  actual  = 0;
    int32_t  status  = USBIP_ST_OK;
    uint32_t isoc_error_count = 0;

    usbip_iso_packet_desc_t *iso_desc = NULL;
    uint32_t *iso_req_offset = NULL;
    uint32_t *iso_req_len = NULL;
    uint32_t *iso_act_len = NULL;
    int32_t  *iso_pkt_status = NULL;

    if (is_isoc) {
        iso_desc = calloc((size_t)num_packets, sizeof(*iso_desc));
        iso_req_offset = calloc((size_t)num_packets, sizeof(*iso_req_offset));
        iso_req_len = calloc((size_t)num_packets, sizeof(*iso_req_len));
        iso_act_len = calloc((size_t)num_packets, sizeof(*iso_act_len));
        iso_pkt_status = calloc((size_t)num_packets, sizeof(*iso_pkt_status));
        if (!iso_desc || !iso_req_offset || !iso_req_len || !iso_act_len || !iso_pkt_status) {
            status = USBIP_ST_ERROR;
            goto send_reply;
        }
    }

    if (ep_num == 0) {
        /* ---- Control transfer (EP0) ---- */
        usb_setup_packet_t setup;
        memcpy(&setup, cmd.setup, sizeof(setup));

        if (dir == USBIP_DIR_OUT && buf_len > 0) {
            if (sock_recv_all(client_fd, data_buf, buf_len) != ESP_OK) {
                free(data_buf);
                return ESP_FAIL;
            }
        }

        uint16_t xfer_actual = 0;
        esp_err_t r = usb_host_driver_ctrl_xfer(&setup, data_buf,
                                                  (uint16_t)buf_len,
                                                  &xfer_actual, xfer_timeout_ms);
        if (r != ESP_OK) {
            ESP_LOGW(TAG, "control xfer failed: %s", esp_err_to_name(r));
            status = esp_err_to_usbip_status(r);
        }
        actual = (int32_t)xfer_actual;
    } else {
        /*
         * Bulk/Interrupt is forwarded to usb_host_driver_ep_xfer().
         * Isochronous currently returns not-supported from the host driver.
         */
        if (dir == USBIP_DIR_OUT && buf_len > 0) {
            /*
             * For OUT transfers, payload follows the submit header/body.
             */
            if (sock_recv_all(client_fd, data_buf, buf_len) != ESP_OK) {
                free(data_buf);
                return ESP_FAIL;
            }
        }

        if (is_isoc && num_packets > 0) {
            if (sock_recv_all(client_fd, iso_desc, (size_t)num_packets * sizeof(*iso_desc)) != ESP_OK) {
                free(data_buf);
                return ESP_FAIL;
            }
            uint64_t total_len = 0;
            for (int32_t i = 0; i < num_packets; i++) {
                iso_req_offset[i] = ntohl(iso_desc[i].offset);
                iso_req_len[i] = ntohl(iso_desc[i].length);
                total_len += iso_req_len[i];
            }
            if (total_len != (uint32_t)buf_len) {
                ESP_LOGW(TAG, "isoc packet lengths (%"PRIu64") != transfer_buffer_length (%"PRId32")",
                         total_len, buf_len);
                status = USBIP_ST_ERROR;
                actual = 0;
                goto send_reply;
            }
        }

        uint8_t ep_addr = (uint8_t)(ep_num & 0x0F);
        if (dir == USBIP_DIR_IN) {
            ep_addr |= 0x80;
        }

        esp_err_t r;
        if (is_isoc) {
            uint32_t xfer_actual = 0;
            r = usb_host_driver_ep_isoc_xfer(ep_addr,
                                             data_buf,
                                             (uint32_t)buf_len,
                                             num_packets,
                                             iso_req_len,
                                             &xfer_actual,
                                             iso_act_len,
                                             iso_pkt_status,
                                             &isoc_error_count,
                                             xfer_timeout_ms);
            actual = (int32_t)xfer_actual;
        } else {
            uint16_t xfer_actual = 0;
            r = usb_host_driver_ep_xfer(ep_addr,
                                        data_buf,
                                        (uint16_t)buf_len,
                                        &xfer_actual,
                                        transfer_flags,
                                        xfer_timeout_ms);
            actual = (int32_t)xfer_actual;
        }
        if (r != ESP_OK) {
            ESP_LOGW(TAG, "non-EP0 xfer failed ep=0x%02x: %s", ep_addr, esp_err_to_name(r));
            status = esp_err_to_usbip_status(r);
            actual = 0;
        } else {
            status = USBIP_ST_OK;
            if (ep_num != 0 && dir == USBIP_DIR_IN && actual == 0) {
                ESP_LOGD(TAG, "seq=%"PRIu32" ep=%"PRIu32" IN short-idle (no data)", seqnum, ep_num);
            }
        }
    }

send_reply:

    /* Build RET_SUBMIT */
    usbip_header_basic_t ret_hdr = {
        .command   = htonl(USBIP_RET_SUBMIT),
        .seqnum    = htonl(seqnum),
        .devid     = 0,
        .direction = 0,
        .ep        = 0,
    };
    usbip_ret_submit_t ret_body = {
        .status           = htonl((uint32_t)status),
        .actual_length    = htonl((uint32_t)actual),
        .start_frame      = 0,
        .number_of_packets = htonl((uint32_t)(is_isoc ? num_packets : -1)),
        .error_count      = htonl(is_isoc ? isoc_error_count : 0),
    };
    memset(ret_body.setup, 0, 8);

    if (sock_send_all(client_fd, &ret_hdr, sizeof(ret_hdr)) != ESP_OK ||
        sock_send_all(client_fd, &ret_body, sizeof(ret_body)) != ESP_OK) {
        ESP_LOGW(TAG, "RET_SUBMIT send failed seq=%"PRIu32, seqnum);
        free(data_buf);
        return ESP_FAIL;
    }

    /* For IN transfers, send the data payload */
    if (dir == USBIP_DIR_IN && actual > 0 && status == USBIP_ST_OK) {
        if (sock_send_all(client_fd, data_buf, actual) != ESP_OK) {
            free(data_buf);
            free(iso_desc);
            free(iso_req_offset);
            free(iso_req_len);
            free(iso_act_len);
            free(iso_pkt_status);
            return ESP_FAIL;
        }
    }

    if (is_isoc && num_packets > 0) {
        for (int32_t i = 0; i < num_packets; i++) {
            iso_desc[i].offset = htonl(iso_req_offset[i]);
            iso_desc[i].length = htonl(iso_req_len[i]);
            iso_desc[i].actual_length = htonl(iso_act_len[i]);
            iso_desc[i].status = htonl((uint32_t)iso_pkt_status[i]);
        }
        if (sock_send_all(client_fd,
                          iso_desc,
                          (size_t)num_packets * sizeof(*iso_desc)) != ESP_OK) {
            free(data_buf);
            free(iso_desc);
            free(iso_req_offset);
            free(iso_req_len);
            free(iso_act_len);
            free(iso_pkt_status);
            return ESP_FAIL;
        }
    }

    free(data_buf);
    free(iso_desc);
    free(iso_req_offset);
    free(iso_req_len);
    free(iso_act_len);
    free(iso_pkt_status);

    ESP_LOGD(TAG, "URB done seq=%"PRIu32" ep=%"PRIu32" dir=%"PRIu32" st=%"PRId32" actual=%"PRId32,
             seqnum, ep_num, dir, status, actual);
    return ESP_OK;
}

/* -------------------------------------------------------------------------
 * Handle USBIP_CMD_UNLINK
 * ---------------------------------------------------------------------- */

static esp_err_t handle_cmd_unlink(int client_fd,
                                    const usbip_header_basic_t *hdr)
{
    usbip_cmd_unlink_t cmd;
    if (sock_recv_all(client_fd, &cmd, sizeof(cmd)) != ESP_OK) {
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "CMD_UNLINK seq=%"PRIu32" unlink_seq=%"PRIu32,
             ntohl(hdr->seqnum), ntohl(cmd.seqnum_unlink));

    usbip_header_basic_t ret_hdr = {
        .command   = htonl(USBIP_RET_UNLINK),
        .seqnum    = htonl(ntohl(hdr->seqnum)),
        .devid     = 0,
        .direction = 0,
        .ep        = 0,
    };
    usbip_ret_unlink_t ret_body = {
        .status = htonl((uint32_t)USBIP_ST_OK),
    };
    memset(ret_body.padding, 0, sizeof(ret_body.padding));

    ESP_RETURN_ON_ERROR(sock_send_all(client_fd, &ret_hdr, sizeof(ret_hdr)),
                        TAG, "send unlink ret hdr");
    ESP_RETURN_ON_ERROR(sock_send_all(client_fd, &ret_body, sizeof(ret_body)),
                        TAG, "send unlink ret body");
    return ESP_OK;
}

/* -------------------------------------------------------------------------
 * Per-client session handler
 * ---------------------------------------------------------------------- */

static void handle_client(int client_fd)
{
    ESP_LOGI(TAG, "new client connected, fd=%d", client_fd);
    bool attached = false;

    /* Keep sessions observable and detect dead peers while debugging */
    struct timeval rx_to = {
        .tv_sec = 2,
        .tv_usec = 0,
    };
    (void)setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &rx_to, sizeof(rx_to));

    int ka = 1;
    (void)setsockopt(client_fd, SOL_SOCKET, SO_KEEPALIVE, &ka, sizeof(ka));

    while (true) {
        if (!attached) {
            /* --- Handshake phase: OP_REQ_DEVLIST or OP_REQ_IMPORT --- */
            op_common_t req;
            esp_err_t rr = sock_recv_all(client_fd, &req, sizeof(req));
            if (rr == ESP_ERR_TIMEOUT) {
                continue;
            }
            if (rr != ESP_OK) {
                ESP_LOGW(TAG, "handshake recv failed, closing session");
                break;
            }
            uint16_t code = ntohs(req.code);

            if (code == OP_REQ_DEVLIST) {
                ESP_LOGI(TAG, "OP_REQ_DEVLIST");
                if (handle_devlist(client_fd) != ESP_OK) break;
            } else if (code == OP_REQ_IMPORT) {
                ESP_LOGI(TAG, "OP_REQ_IMPORT");
                if (handle_import(client_fd, &attached) != ESP_OK) break;
            } else {
                ESP_LOGW(TAG, "unknown OP code: 0x%04x", code);
                break;
            }
        } else {
            /* --- URB phase: CMD_SUBMIT or CMD_UNLINK --- */
            usbip_header_basic_t hdr;
            esp_err_t rr = sock_recv_all(client_fd, &hdr, sizeof(hdr));
            if (rr == ESP_ERR_TIMEOUT) {
                ESP_LOGD(TAG, "URB phase idle: waiting for next command");
                continue;
            }
            if (rr != ESP_OK) {
                ESP_LOGW(TAG, "URB header recv failed, closing session");
                break;
            }
            uint32_t cmd = ntohl(hdr.command);
            ESP_LOGD(TAG, "URB cmd=0x%08"PRIx32" seq=%"PRIu32" ep=%"PRIu32" dir=%"PRIu32,
                     cmd, ntohl(hdr.seqnum), ntohl(hdr.ep), ntohl(hdr.direction));
            if (cmd == USBIP_CMD_SUBMIT) {
                if (handle_cmd_submit(client_fd, &hdr) != ESP_OK) {
                    ESP_LOGW(TAG, "CMD_SUBMIT handler failed, closing session");
                    break;
                }
            } else if (cmd == USBIP_CMD_UNLINK) {
                if (handle_cmd_unlink(client_fd, &hdr) != ESP_OK) {
                    ESP_LOGW(TAG, "CMD_UNLINK handler failed, closing session");
                    break;
                }
            } else {
                ESP_LOGW(TAG, "unknown USBIP command: 0x%08"PRIx32, cmd);
                break;
            }
        }
    }

    ESP_LOGI(TAG, "client session ended");
    close(client_fd);
}

/* -------------------------------------------------------------------------
 * Server task
 * ---------------------------------------------------------------------- */

static bool s_running = false;
static int  s_listen_fd = -1;

static void usbip_server_task(void *arg)
{
    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_port        = htons(CONFIG_USBIP_SERVER_PORT),
    };

    s_listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s_listen_fd < 0) {
        ESP_LOGE(TAG, "socket() failed: %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(s_listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (bind(s_listen_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "bind() failed: %d", errno);
        close(s_listen_fd);
        vTaskDelete(NULL);
        return;
    }

    if (listen(s_listen_fd, 1) != 0) {
        ESP_LOGE(TAG, "listen() failed: %d", errno);
        close(s_listen_fd);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "USB/IP server listening on port %d", CONFIG_USBIP_SERVER_PORT);

    while (s_running) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client_fd = accept(s_listen_fd,
                               (struct sockaddr *)&client_addr, &client_len);
        if (client_fd < 0) {
            if (s_running) {
                ESP_LOGE(TAG, "accept() failed: %d", errno);
            }
            break;
        }

        char ip_str[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &client_addr.sin_addr, ip_str, sizeof(ip_str));
        ESP_LOGI(TAG, "accepted connection from %s", ip_str);

        handle_client(client_fd);
    }

    close(s_listen_fd);
    s_listen_fd = -1;
    vTaskDelete(NULL);
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

esp_err_t usbip_server_start(void)
{
    s_running = true;
    BaseType_t ret = xTaskCreate(usbip_server_task, "usbip_srv",
                                  8192, NULL, 5, NULL);
    return (ret == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
}

void usbip_server_stop(void)
{
    s_running = false;
    if (s_listen_fd >= 0) {
        close(s_listen_fd);
    }
}
