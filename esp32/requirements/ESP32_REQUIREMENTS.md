# ESP32 Firmware — Requirements Document

---

## 1. Overview

The ESP32 acts as a **WiFi bridge** between the drone's flight controller (FC) and the Ground Control Station (GCS).

It plays two roles simultaneously:

- **Sensor gateway** — reads peripherals directly (GPS, MTF-01 optical flow, radio receiver, barometer) and formats their data into the shared UDP protocol
- **FC bridge** — receives high-frequency telemetry from the flight controller (attitude, motor states, FSM state) via SPI, and forwards GCS commands (PID updates, calibration) back to the FC via the same SPI bus

```
┌─────────────────────────────────────────────────────┐
│                      ESP32                          │
│                                                     │
│  ┌──────────┐   SPI slave    ┌──────────────────┐  │
│  │   WiFi   │◄──────────────►│ Flight Controller│  │
│  │  UDP GCS │                │  (STM32 / etc.)  │  │
│  └──────────┘                └──────────────────┘  │
│       ▲                                             │
│       │ assemble & send                             │
│  ┌────┴─────────────────────────────────────────┐  │
│  │ Sensor readers                               │  │
│  │  • GPS      (UART2)                          │  │
│  │  • MTF-01   (UART1)                          │  │
│  │  • S.Bus    (UART0 inverted)                 │  │
│  │  • Barometer (I2C)                           │  │
│  └──────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────┘
```

---

## 2. Shared Protocol

The UDP packet format is **identical** to the one used by the GCS.
The authoritative struct definitions live in `src/backend/Protocol.h` (C++) and are reused directly in the ESP32 firmware.

The ESP32 firmware must include (or duplicate) this header without modification.
Any change to `Protocol.h` must be reflected in the firmware.

See `src/backend/Protocol.h` and `requirements/REQUIREMENTS.md` §4 for the full packet specification.

---

## 3. Hardware — Pinout

### 3.1 SPI (slave — Flight Controller link)

| Signal | GPIO |
|--------|------|
| MOSI   | 13   |
| MISO   | 12   |
| SCK    | 14   |
| CS     | 27   |

The FC is the **SPI master**. The ESP32 is the **SPI slave**.

**Direction:**
- FC → ESP32 (MOSI): attitude quaternion, gyro, accel, motor states, FSM state (high-frequency telemetry)
- ESP32 → FC (MISO): PID update commands, calibration commands received from GCS

### 3.2 MTF-01 Optical Flow + Laser Rangefinder (UART)

| Signal | GPIO |
|--------|------|
| RX     | 25   |
| TX     | 26   |

- Baud rate: TBD (refer to MTF-01 datasheet)
- Data: distance (m), optical flow X/Y (px/s), quality (0–255)
- Expected output rate: 50 Hz
- Maps to `PKT_MTF01 (0x03)`

### 3.3 GPS (UART)

| Signal | GPIO |
|--------|------|
| RX     | 2    |
| TX     | 4    |

- Protocol: NMEA 0183 or UBX (depending on GPS module)
- Baud rate: 9600 or 115200 (module-dependent)
- Fields used: latitude, longitude, altitude, speed, heading, satellites, fix type
- Expected output rate: 10 Hz
- Maps to `PKT_GPS (0x02)`

### 3.4 Radio Receiver — S.Bus (inverted UART)

| Signal | GPIO |
|--------|------|
| RX     | 16   |
| TX     | 17   |

- S.Bus is **inverted UART**: 100000 baud, 8E2, signal is logically inverted
- The ESP32 hardware UART supports inversion via `UART_SIGNAL_RXD_INV`
- 16 channels decoded, channels 1–8 forwarded to GCS
- RSSI derived from channel 17 (failsafe flags) or hardcoded from WiFi RSSI
- Maps to `PKT_RADIO (0x04)`

### 3.5 Barometer (I2C)

| Signal | GPIO |
|--------|------|
| SCL    | 22   |
| SDA    | 21   |

- Standard ESP32 I2C pins (Wire library default)
- Supported IC: BMP280 / BMP388 / MS5611 (TBD — to be confirmed with hardware)
- Fields: pressure (Pa), temperature (°C), altitude (m, computed from pressure using ISA formula)
- Expected output rate: 10 Hz
- Maps to `PKT_BARO (0x08)`

---

## 4. WiFi & UDP

- Mode: **Station (STA)** — connects to a known WiFi network (SSID + password stored in config)
- GCS IP and port: configurable (default port 5005)
- The ESP32 discovers the GCS by sending telemetry to the configured GCS address
- Commands from the GCS arrive on the same UDP socket (source port = GCS port)
- Connection considered lost if no UDP keepalive/command is received for 2 s (optional heartbeat)

---

## 5. Software Architecture

### 5.1 Tasks (FreeRTOS)

| Task | Core | Priority | Role |
|------|------|----------|------|
| `task_spi`     | 1 | High   | SPI slave ISR + ring buffer drain; pushes FC telemetry to queues |
| `task_gps`     | 0 | Normal | NMEA/UBX parse; pushes `GpsData` to queue |
| `task_mtf01`   | 0 | Normal | MTF-01 UART parse; pushes `Mtf01Data` to queue |
| `task_sbus`    | 0 | Normal | S.Bus frame decode; pushes `RadioData` to queue |
| `task_baro`    | 0 | Normal | I2C poll at 10 Hz; pushes `BaroData` to queue |
| `task_udp_tx`  | 0 | Normal | Assembles packets from all queues, sends UDP datagrams at up to 100 Hz |
| `task_udp_rx`  | 0 | Normal | Receives UDP from GCS, forwards commands to FC via SPI MISO buffer |

### 5.2 Packet Send Rates

| Packet | Rate | Source |
|--------|------|--------|
| `PKT_ATTITUDE (0x01)` | 100 Hz | FC via SPI |
| `PKT_GPS (0x02)`      | 10 Hz  | GPS UART |
| `PKT_MTF01 (0x03)`    | 50 Hz  | MTF-01 UART |
| `PKT_RADIO (0x04)`    | 50 Hz  | S.Bus UART |
| `PKT_STATUS (0x05)`   | 10 Hz  | FC via SPI |
| `PKT_PID (0x06)`      | 1 Hz   | FC via SPI (on change) |
| `PKT_LOG (0x07)`      | On event | FC via SPI or ESP32 internal |
| `PKT_BARO (0x08)`     | 10 Hz  | Barometer I2C |
| `PKT_CALIB_STATUS (0x09)` | On event | FC via SPI |

### 5.3 Command Forwarding (GCS → FC)

| Packet received from GCS | Action |
|--------------------------|--------|
| `PKT_SET_PID (0x10)`     | Write into SPI MISO buffer; FC reads it on next SPI transaction |
| `PKT_CALIB_CMD (0x20)`   | Same — write into SPI MISO buffer |

The ESP32 sends an `PKT_ACK (0x11)` back to the GCS once the command has been transferred to the FC (or immediately upon receipt, depending on FC handshake design — TBD).

### 5.4 SPI Slave Protocol (FC ↔ ESP32)

The SPI frame format between FC and ESP32 is a **separate, lower-level protocol** (not the GCS UDP protocol). It is to be defined, but must convey:

**FC → ESP32 (MOSI, each SPI transaction):**
- Attitude quaternion + gyro + accel
- Motor states
- FSM state string
- Battery voltage/current/percent
- Calibration status (per target)
- PID values (periodically)
- Log messages (on event)

**ESP32 → FC (MISO, piggy-backed on each transaction):**
- Pending GCS command (if any): type + payload (PID update or CalibCmd)
- Command present flag (1 bit or first byte) to avoid FC parsing stale data

---

## 6. Configuration

The following parameters must be configurable without recompiling (stored in NVS / EEPROM):

| Parameter | Default | Description |
|-----------|---------|-------------|
| WiFi SSID | — | Network to connect to |
| WiFi password | — | Network password |
| GCS IP | 0.0.0.0 (broadcast) | Target IP for UDP datagrams |
| GCS port | 5005 | Target UDP port |
| GPS baud rate | 115200 | Matches GPS module config |
| MTF-01 baud rate | TBD | Matches sensor config |
| Barometer I2C address | 0x76 | BMP280 default; 0x77 if SDO pulled high |

---

## 7. Non-Functional Requirements

- All sensor reads and SPI transactions must be **non-blocking** relative to the UDP transmit task
- The WiFi stack must not starve sensor tasks — use FreeRTOS task priorities accordingly
- Watchdog timer enabled: reset if `task_udp_tx` stalls for more than 2 s
- Serial monitor (USB UART) outputs human-readable status at 1 Hz for debugging
- Firmware built with Arduino framework + ESP32 Arduino Core (or ESP-IDF — TBD)

---

## 8. Out of Scope (for now)

- OTA firmware update
- Flight controller attitude computation (done on the FC, not the ESP32)
- Data logging to SD card
- Encrypted UDP
