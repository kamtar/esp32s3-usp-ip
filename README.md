# ESP32-S3 USB/IP Host (Experimental)

USB/IP host firmware for ESP32-S3 that exports one attached USB device over Wi-Fi.

This project is focused on practical bring-up and debugging (especially CDC/UART bridge devices like CP210x) and includes Wi-Fi log streaming so you can debug without USB serial.

## Status

- Target: `esp32s3` (USB Host supported)
- USB/IP handshake: implemented (`DEVLIST`, `IMPORT`)
- Transfer support:
  - Control (EP0): implemented
  - Bulk/Interrupt: implemented
  - Isochronous: implemented (basic)
- Logging over Wi-Fi: implemented (TCP/telnet log server)
- Current architecture: single exported device (`1-1`)

Notes:
- This is still experimental firmware.
- Some host/client combinations may still require further work (especially high-load CDC/open/close edge cases).

## Repository Layout

- `main.c`: app startup flow (Wi-Fi -> logging -> USB host -> USB/IP server)
- `wifi_manager.c`: Wi-Fi station connect/disconnect
- `wifi_log_sink.c`: TCP log server over Wi-Fi
- `usb_host_driver.c`: ESP-IDF USB Host wrapper (descriptor parse, control/bulk/int/iso transfers)
- `usbip_server.c`: USB/IP protocol server and URB processing
- `include/`: public headers
- `Kconfig.projbuild`: project settings

## Architecture Diagrams (GitHub Mermaid)

The diagrams below are rendered by GitHub in Markdown preview.

### 1) Firmware Runtime Topology

```mermaid
flowchart LR
  subgraph PC[PC / USBIP Client]
    C[usbip tool or driver]
    T[Optional telnet or nc]
  end

  subgraph ESP[ESP32-S3 Firmware]
    M[main.c\nboot orchestration]
    W[wifi_manager.c\nWi-Fi STA connect]
    L[wifi_log_sink.c\nTCP log mirror]
    S[usbip_server.c\nTCP 3240 server\nUSBIP protocol]
    H[usb_host_driver.c\nESP-IDF USB Host wrapper]
  end

  U[(Attached USB Device)]

  M --> W
  M --> L
  M --> H
  M --> S

  C -->|DEVLIST / IMPORT / URBs| S
  S -->|control, bulk/int, isoc requests| H
  H -->|USB transactions| U
  H -->|descriptors + transfer results| S
  L -->|log stream TCP 23| T
```

### 2) USB/IP Handshake + URB Loop

```mermaid
sequenceDiagram
  participant Client as USBIP Client
  participant Server as usbip_server.c
  participant Host as usb_host_driver.c
  participant USB as USB Device

  Client->>Server: TCP connect :3240
  Client->>Server: OP_REQ_DEVLIST
  Server->>Host: usb_host_driver_get_snapshot()
  Host-->>Server: attached device snapshot (or none)
  Server-->>Client: OP_REP_DEVLIST (+ iface stubs)

  Client->>Server: OP_REQ_IMPORT("1-1")
  Server->>Host: usb_host_driver_get_snapshot()
  Host-->>Server: validate exported device
  Server-->>Client: OP_REP_IMPORT(status)

  loop While client attached
    Client->>Server: USBIP_CMD_SUBMIT(seq, ep, dir, setup/payload)
    alt EP0 control
      Server->>Host: usb_host_driver_ctrl_xfer(...)
      Host->>USB: Control transfer
      USB-->>Host: Data/status
      Host-->>Server: status + actual
    else Non-EP0 bulk/intr
      Server->>Host: usb_host_driver_ep_xfer(...)
      Host->>USB: Endpoint transfer
      USB-->>Host: Data/status
      Host-->>Server: status + actual
    else Non-EP0 isoc
      Server->>Host: usb_host_driver_ep_isoc_xfer(...)
      Host->>USB: Iso transfer + packet descriptors
      USB-->>Host: per-packet status
      Host-->>Server: status + actual + packet results
    end
    Server-->>Client: USBIP_RET_SUBMIT (+ IN payload if any)
  end

  opt Cancellation
    Client->>Server: USBIP_CMD_UNLINK
    Server-->>Client: USBIP_RET_UNLINK
  end
```

### 3) Per-Client Session State Machine

```mermaid
stateDiagram-v2
  [*] --> Listening: usbip_server_start()
  Listening --> Connected: accept()

  state Connected {
    [*] --> Handshake
    Handshake --> Handshake: OP_REQ_DEVLIST / send OP_REP_DEVLIST
    Handshake --> Attached: OP_REQ_IMPORT success
    Handshake --> Closed: bad opcode or no device

    Attached --> Attached: CMD_SUBMIT / RET_SUBMIT
    Attached --> Attached: CMD_UNLINK / RET_UNLINK
    Attached --> Closed: recv/send error or disconnect
  }

  Connected --> Listening: session end, close(client_fd)
  Listening --> [*]: usbip_server_stop()
```

### 4) CMD_SUBMIT Dispatch Logic

```mermaid
flowchart TD
  A[Receive USBIP_CMD_SUBMIT] --> B{Valid len/packet fields?}
  B -- no --> X[Fail session]
  B -- yes --> C{EP == 0?}

  C -- yes --> D[Parse setup\nrecv OUT payload if any]
  D --> E[usb_host_driver_ctrl_xfer]
  E --> F[Build RET_SUBMIT\nstatus + actual]

  C -- no --> G{Isochronous?\nnumber_of_packets >= 0}
  G -- no --> H[recv OUT payload if any]
  H --> I[usb_host_driver_ep_xfer]
  I --> F

  G -- yes --> J[recv isoc desc + payload]
  J --> K[usb_host_driver_ep_isoc_xfer]
  K --> L[append per-packet desc]
  L --> F

  F --> M{IN and actual > 0 and status OK?}
  M -- yes --> N[Send IN payload]
  M -- no --> O[Done]
  N --> O
```

## Requirements

- ESP-IDF v5.5 (or compatible with this project)
- ESP32-S3 board with USB OTG host wiring/power
- Stable 5V power path for USB device side (do not power USB peripherals from weak rails)

## Configure

Use ESP-IDF menuconfig (or edit `sdkconfig` through menuconfig):

- Wi-Fi:
  - `USBIP_WIFI_SSID`
  - `USBIP_WIFI_PASSWORD`
- USB/IP:
  - `USBIP_SERVER_PORT` (default 3240)
- Logging:
  - `USBIP_WIFI_LOG_ENABLE` (enable TCP log server)
  - `USBIP_WIFI_LOG_PORT` (default 23)
  - `USBIP_DEBUG_LOGS` (enable verbose debug-level tags)

## Build / Flash

### With ESP-IDF extension (recommended in this workspace)

- Build: use ESP-IDF `build`
- Flash: use ESP-IDF `flash`
- Monitor (if serial available): use ESP-IDF `monitor`

### CLI (if using idf.py)

From project root:

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p <PORT> flash
```

## Use USB/IP

Assume ESP IP is `192.168.0.213`.

### Linux / WSL

```bash
usbip list -r 192.168.0.213
usbip attach -r 192.168.0.213 -b 1-1
```

### Windows

Use your USB/IP client tooling to:

- list remote devices from `192.168.0.213:3240`
- attach bus ID `1-1`

## Wi-Fi Log Streaming (No USB Serial Needed)

When `USBIP_WIFI_LOG_ENABLE=y`, firmware starts a TCP log server.

Connect from PC:

```bash
telnet 192.168.0.213 23
```

If Telnet is unavailable, any TCP client (`nc`, PuTTY raw/telnet mode) works.

## Firmware Editing Guide

### Change USB/IP protocol behavior

Edit `usbip_server.c`:

- handshake (`OP_REQ_DEVLIST`, `OP_REQ_IMPORT`)
- URB command handling (`CMD_SUBMIT`, `CMD_UNLINK`)
- network send/receive behavior and session lifecycle

### Change USB transfer implementation

Edit `usb_host_driver.c`:

- endpoint/interface discovery
- control/bulk/int/iso transfer submission and timeout cleanup
- host callback and event handling

### Change debug behavior

- `main.c`: runtime log level setup
- `Kconfig.projbuild`: expose toggles
- `wifi_log_sink.c`: network log transport

## Known Limits / Design Constraints

- Single exported device model (one `busid`)
- No full multi-device/hub manager yet
- UNLINK behavior is basic and may need deeper async in-flight URB tracking for best stability

## Safety / Hardware Notes

- Use a robust 5V supply path for USB host mode.
- Prefer powered USB hub for power-hungry peripherals.
- Avoid hot-plug stress during unstable bring-up.
- If board browns out or overheats, stop and verify wiring/power immediately.

## Quick Resume Checklist (When Returning to This)

1. Verify board power path and OTG wiring first.
2. Build + flash latest firmware.
3. Connect Wi-Fi log telnet (`<esp_ip>:23`).
4. Verify `DEVLIST` and `IMPORT` flow.
5. Re-test attach/open sequence for your target device.
6. Capture logs around the first stall/disconnect and iterate in `usbip_server.c` + `usb_host_driver.c`.

## License

MIT (per source headers).