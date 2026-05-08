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
The authoritative struct definitions live in `common/Protocol.h` at the repository root and are included directly by both the GCS and the ESP32 firmware — there is only one copy.

The ESP32 build system must add `<repo_root>/common` to the include path so that `#include "Protocol.h"` resolves to this shared file.
Any change to `common/Protocol.h` affects both projects simultaneously.

See `common/Protocol.h` and `requirements/REQUIREMENTS.md` §4 for the full packet specification.

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

- Baud rate: **115200 baud**, 8N1
- Protocol: **MAVLink v1** (see §5.5 for full parsing specification)
- Data: distance (m), optical flow X/Y (px/s), quality (0–255)
- Expected output rate: ~50 Hz
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

### 5.1 Execution Model — Bare Metal C++

The firmware is written in **C++17, bare metal** — no FreeRTOS, no OS abstraction.
Framework: **Arduino Core for ESP32** (provides `setup()` / `loop()`, hardware drivers, WiFi stack).

All concurrency is handled with:
- **Hardware interrupts (ISR)** — SPI slave CS edge, UART RX for GPS / MTF-01 / S.Bus
- **Ring buffers** — ISRs write raw bytes; `loop()` drains and parses
- **Non-blocking polling** — each driver exposes a `update()` method called every loop iteration; no busy-wait, no blocking I/O

`loop()` executes at the fastest possible rate (~100 Hz target). Each subsystem is called in sequence:

```
loop()
├── spi_slave.update()     // drain SPI RX ring buffer, parse FC frame
├── gps.update()           // drain GPS UART ring buffer, parse NMEA/UBX
├── mtf01.update()         // drain MTF-01 UART ring buffer
├── sbus.update()          // drain S.Bus UART ring buffer, decode frame
├── baro.update()          // I2C poll if 100 ms elapsed (10 Hz)
├── udp_rx.update()        // receive pending GCS commands (non-blocking)
└── udp_tx.update()        // send pending telemetry packets (rate-limited per type)
```

### 5.2 Module List

| Module | File | Role |
|--------|------|------|
| `SpiSlave`   | `spi_slave.h/.cpp`  | SPI slave ISR + frame parser; exposes latest FC telemetry |
| `GpsReader`  | `gps_reader.h/.cpp` | UART2 NMEA/UBX parser; exposes `GpsData` |
| `Mtf01Reader`| `mtf01.h/.cpp`      | UART1 binary parser; exposes `Mtf01Data` |
| `SbusReader` | `sbus.h/.cpp`       | UART0 inverted, S.Bus frame decoder; exposes `RadioData` |
| `BaroReader` | `baro.h/.cpp`       | I2C poller (BMP280/388/MS5611); exposes `BaroData` |
| `UdpTx`      | `udp_tx.h/.cpp`     | Rate-limited packet builder and sender |
| `UdpRx`      | `udp_rx.h/.cpp`     | Non-blocking command receiver; writes pending command to SPI MISO buffer |
| `Config`     | `config.h/.cpp`     | NVS read/write for all configurable parameters |

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

### 5.5 MTF-01 MAVLink v1 Parser

#### 5.5.1 Wire format

The MTF-01 speaks **MAVLink v1** at 115200 baud, 8N1 on UART (GPIO 25 RX / 26 TX).
Each MAVLink v1 frame has the following byte layout:

```
Offset  Size  Field
  0      1    STX   — always 0xFE (start-of-frame marker)
  1      1    LEN   — payload length in bytes
  2      1    SEQ   — sender sequence number (wraps 0–255)
  3      1    SYSID — system ID of the sender
  4      1    COMPID— component ID of the sender
  5      1    MSGID — message type identifier
  6    LEN    PAYLOAD
6+LEN    2    CRC   — little-endian CRC16-X25 (see §5.5.3)
```

Total frame size: `LEN + 8` bytes.

#### 5.5.2 Relevant message types

Only two message IDs are parsed; all others are discarded.

---

**MSG ID 100 — OPTICAL_FLOW** (payload = 26 bytes)

| Offset in payload | Type    | Field     | Unit   | Notes |
|-------------------|---------|-----------|--------|-------|
| 0–3               | float   | time_usec | (unused) | |
| 4–5               | int16   | flow_comp_m_x | (unused) | |
| 6–7               | int16   | flow_comp_m_y | (unused) | |
| 8–11              | float   | ground_distance | (unused) | |
| 12–13             | int16   | flow_x    | dpix/s | raw optical flow X |
| 14–15             | int16   | flow_y    | dpix/s | raw optical flow Y |  
| 16                | uint8   | sensor_id | (unused) | |
| 17                | uint8   | quality   | 0–255  | flow confidence |

Wait, I need to cross-check. Let me use the exact offsets you provided:
- flowX: int16 at payload bytes 20–21
- flowY: int16 at payload bytes 22–23
- quality: uint8 at payload byte 25

| Offset in payload | Type   | Field   | Used | Notes |
|-------------------|--------|---------|------|-------|
| 0–19              | —      | (various) | No | ignored |
| 20–21             | int16  | flow_x  | Yes | raw optical flow X, dpix/s |
| 22–23             | int16  | flow_y  | Yes | raw optical flow Y, dpix/s |
| 24                | —      | —       | No  | ignored |
| 25                | uint8  | quality | Yes | 0 = no confidence, 255 = max |

CRC extra byte: **175** (0xAF)

---

**MSG ID 132 — DISTANCE_SENSOR** (payload = 14 bytes)

| Offset in payload | Type   | Field            | Used | Notes |
|-------------------|--------|------------------|------|-------|
| 0–7               | —      | (various)        | No   | ignored |
| 8–9               | uint16 | current_distance | Yes  | in centimetres |
| 10–13             | —      | (various)        | No   | ignored |

Conversion: `distance_m = current_distance * 0.01f`

CRC extra byte: **85** (0x55)

---

#### 5.5.3 CRC16-X25 calculation

The checksum covers bytes `[LEN, SEQ, SYSID, COMPID, MSGID, PAYLOAD...]` (everything after STX, excluding the CRC bytes themselves), followed by one **message-specific extra byte** (also called `MAVLINK_MSG_CRC` or magic CRC):

```
crc = crc16_x25_init()           // 0xFFFF
for each byte b in [LEN..PAYLOAD]:
    crc = crc16_x25_update(crc, b)
crc = crc16_x25_update(crc, extra_byte)   // 175 for MSG 100, 85 for MSG 132
```

CRC16-X25 polynomial: `0x1021`, reflected (LSB-first), init `0xFFFF`.
The two CRC bytes in the frame are stored **little-endian** (CRC_L first, CRC_H second).

#### 5.5.4 Parser state machine

The parser processes incoming bytes one at a time using an 8-state machine:

```
WAIT_STX → LEN → SEQ → SYSID → COMPID → MSGID → PAYLOAD → CRC_L → CRC_H
              ↑                                                         |
              └──────────────── bad CRC or unknown MSGID ──────────────┘
                                (reset to WAIT_STX)
```

| State    | Action |
|----------|--------|
| `WAIT_STX` | Wait for byte `0xFE`; advance to `LEN` |
| `LEN`    | Store payload length; reject if > 255; advance to `SEQ`; reset CRC accumulator and feed `LEN` into it |
| `SEQ`    | Store seq; feed into CRC; advance to `SYSID` |
| `SYSID`  | Store sysid; feed into CRC; advance to `COMPID` |
| `COMPID` | Store compid; feed into CRC; advance to `MSGID` |
| `MSGID`  | Store msgid; feed into CRC; if msgid ∉ {100, 132}: reset to `WAIT_STX`; else advance to `PAYLOAD` |
| `PAYLOAD`| Accumulate bytes into a fixed 26-byte buffer; feed into CRC; after `LEN` bytes advance to `CRC_L` |
| `CRC_L`  | Store low CRC byte; advance to `CRC_H` |
| `CRC_H`  | Store high CRC byte; feed extra byte into CRC; compare with accumulated CRC; on match dispatch message; reset to `WAIT_STX` |

Payload buffer is always 26 bytes (size of the largest message). Bytes beyond the declared `LEN` are never read.

#### 5.5.5 Output and rate

On each valid frame:
- **MSG 100**: update `flow_x`, `flow_y`, `quality` in the latest `Mtf01Data` snapshot
- **MSG 132**: update `distance_m` in the latest `Mtf01Data` snapshot
- After either update, set a `newData` flag

The MTF-01 sends both messages at approximately 50 Hz each. The `Mtf01Reader::update()` method drains the UART ring buffer and calls the state machine for every available byte. The GCS packet `PKT_MTF01 (0x03)` is built and sent whenever `newData` is true.

#### 5.5.6 Interface (`Mtf01Reader` class)

```cpp
class Mtf01Reader {
public:
    void begin(HardwareSerial& serial, uint32_t baud); // call once in setup()
    void update();           // drain UART RX buffer, run state machine
    bool     newData() const;
    float    distance() const;   // metres
    int16_t  flowX()    const;   // dpix/s
    int16_t  flowY()    const;   // dpix/s
    uint8_t  quality()  const;   // 0–255
};
```

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
- Firmware built with **Arduino Core for ESP32**, C++17, bare metal (no FreeRTOS)

---
