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

In the Arduino sketch, the file is included via a relative path:
```cpp
#include "../../../common/Protocol.h"
```

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

- Bus: **SPI2\_HOST (HSPI)**
- Mode: **0** (CPOL=0, CPHA=0) — must match the FC master configuration
- Frame size: **256 bytes fixed** in both directions
- The FC is the **SPI master**. The ESP32 is the **SPI slave**.

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

- Standard ESP32 I2C pins (Wire library default), shared with the BN-880 compass
- Fitted IC: **BMP180** (Bosch), fixed I2C address `0x77`
- Driver: `Bmp180Reader` (`Bmp180Reader.h/.cpp`) — reads the 11 factory calibration
  coefficients at start-up, then runs the datasheet fixed-point compensation to
  produce true temperature and pressure
- Fields: pressure (Pa), temperature (°C), altitude (m, computed from pressure using ISA formula)
- Output rate: 5 Hz (blocking conversion ~9 ms, throttled in `loop()`)
- Maps to `PKT_BARO (0x08)`
- **Current path: direct ESP32 → GCS** (first debug pass). Unlike GPS / MTF-01 /
  magnetometer it does not yet go through the FC over SPI; this is intentional
  for bring-up and should later follow the same SPI round trip.

---

## 4. WiFi & UDP

### 4.1 Connection strategy

1. On boot, attempt to join the configured WiFi network (STA mode). Timeout: **10 seconds**.
2. If connection fails, fall back to **Access Point mode**:
   - SSID: `MicroFlight-ESP32`
   - Password: `microflight`
   - ESP32 IP in AP mode: `192.168.4.1`

### 4.2 UDP transmission

- Telemetry is sent to the **subnet broadcast address** (e.g. `192.168.1.255`) on port **5005**.
- The broadcast approach removes the need to configure the GCS IP — any GCS on the same network receives the packets.
- In AP mode, broadcasts go to `192.168.4.255`.

### 4.3 Heartbeat (no FC connected)

When the SPI link to the FC is inactive, the ESP32 still sends two packets per second to keep the GCS alive:

| Packet | Rate | Content |
|--------|------|---------|
| `PKT_STATUS` | 1 Hz | `state = "NO-FC"` (or `"SPI-ERR"` if init failed), `wifi_rssi` mapped from dBm |
| `PKT_LOG`    | 1 Hz | SPI throughput stats: `[SPI] att=0/s  sta=0/s  other=0/s` |

### 4.4 Credentials

WiFi credentials are stored in `Secrets.h` in the sketch folder. This file is listed in `.gitignore` and must never be committed. A `Secrets.h.example` template is versioned instead.

```cpp
// Secrets.h
static const char* WIFI_SSID     = "your_network";
static const char* WIFI_PASSWORD = "your_password";
```

---

## 5. Software Architecture

### 5.1 Execution Model — Bare Metal C++

The firmware is written in **C++17, bare metal** — no FreeRTOS, no OS abstraction.
Framework: **Arduino Core for ESP32** (provides `setup()` / `loop()`, hardware drivers, WiFi stack).

All concurrency is handled with:
- **Hardware interrupts (ISR)** — SPI slave CS edge, UART RX for GPS / MTF-01 / S.Bus
- **Ring buffers** — ISRs write raw bytes; `loop()` drains and parses
- **Non-blocking polling** — each driver exposes an `update()` method called every loop iteration; no busy-wait, no blocking I/O

`loop()` executes at the fastest possible rate (~100 Hz target). Each subsystem is called in sequence:

```
loop()
├── g_spi.update()         // drain completed SPI transaction; forward PKT_ATTITUDE to GCS
├── gps.update()           // drain GPS UART ring buffer, parse NMEA/UBX      [not yet impl.]
├── mtf01.update()         // drain MTF-01 UART ring buffer                   [not yet impl.]
├── sbus.update()          // drain S.Bus UART ring buffer, decode frame       [not yet impl.]
├── g_baro.update()        // I2C poll at 5 Hz; sendBaro() direct to GCS (PKT_BARO)
├── udp_rx.update()        // receive pending GCS commands (non-blocking)     [not yet impl.]
└── [1 Hz] sendStatus() + sendLog()   // heartbeat — always runs
```

### 5.2 Module List

| Module | File | Status | Role |
|--------|------|--------|------|
| `SpiSlave`    | `SpiSlave.h/.cpp`   | ✅ Implemented | SPI slave driver; parses FC frames, exposes telemetry, holds pending commands |
| `GpsReader`   | `gps_reader.h/.cpp` | ⬜ Planned | UART2 NMEA/UBX parser; exposes `GpsData` |
| `Mtf01Reader` | `Mtf01Reader.h/.cpp`| ⬜ Planned | UART1 MAVLink v1 parser; exposes `Mtf01Data` |
| `SbusReader`  | `sbus.h/.cpp`       | ⬜ Planned | UART0 inverted, S.Bus frame decoder; exposes `RadioData` |
| `Bmp180Reader`| `Bmp180Reader.h/.cpp`| ✅ Implemented | BMP180 I2C poller (0x77); calibration + datasheet compensation; direct `PKT_BARO` send |
| `UdpRx`       | *(in .ino)*         | ⬜ Planned | Non-blocking command receiver; calls `SpiSlave::setPendingCommand()` |

### 5.3 Packet Send Rates

| Packet | Rate | Source | Status |
|--------|------|--------|--------|
| `PKT_ATTITUDE (0x01)` | 100 Hz | FC via SPI | ✅ Implemented |
| `PKT_GPS (0x02)`      | 10 Hz  | GPS UART | ⬜ Planned |
| `PKT_MTF01 (0x03)`    | 50 Hz  | MTF-01 UART | ⬜ Planned |
| `PKT_RADIO (0x04)`    | 50 Hz  | S.Bus UART | ⬜ Planned |
| `PKT_STATUS (0x05)`   | 10 Hz (FC) / 1 Hz (heartbeat) | FC via SPI / ESP32 internal | ✅ Heartbeat implemented |
| `PKT_PID (0x06)`      | 1 Hz   | FC via SPI | ⬜ Planned |
| `PKT_LOG (0x07)`      | On event / 1 Hz heartbeat | FC via SPI / ESP32 internal | ✅ Heartbeat implemented |
| `PKT_BARO (0x08)`     | 5 Hz   | BMP180 I2C (direct, debug) | ✅ Implemented |
| `PKT_CALIB_STATUS (0x09)` | On event | FC via SPI | ⬜ Planned |

### 5.4 Command Forwarding (GCS → FC)

| Packet received from GCS | Action |
|--------------------------|--------|
| `PKT_SET_PID (0x10)`     | Write into SPI MISO buffer; FC reads it on next SPI transaction |
| `PKT_CALIB_CMD (0x20)`   | Same — write into SPI MISO buffer |

`SpiSlave::setPendingCommand()` is implemented and ready. UDP RX (receiving commands from the GCS) is not yet wired up.

The ESP32 will send `PKT_ACK (0x11)` back to the GCS immediately upon receiving a command (before the FC confirms it).

### 5.5 SPI Slave Protocol (FC ↔ ESP32)

The SPI frame format is defined in `SpiFrame.h` (in the sketch folder). It is a **separate, lower-level protocol** from the GCS UDP protocol.

Both directions use **256-byte fixed frames** so the FC always clocks the exact same number of bytes per transaction.

#### 5.5.1 FC → ESP32 frame (MOSI)

```
[SpiFrameHeader 4B] [payload up to 248B] [CRC16 2B] [pad 2B]
```

**`SpiFrameHeader` (4 bytes):**

| Field | Type | Value |
|-------|------|-------|
| `magic` | uint16 | `0xBEEF` |
| `type` | uint8 | Frame type (see below) |
| `payload_len` | uint8 | Valid bytes after the header |

**Frame types:**

| Type | Value | Rate | Payload struct |
|------|-------|------|----------------|
| `Attitude`    | 0x01 | 100 Hz   | `SpiPayloadAttitude` (40 B): qw, qx, qy, qz, gx, gy, gz, ax, ay, az |
| `Status`      | 0x02 |  10 Hz   | `SpiPayloadStatus` (50 B): battery V/A/%, state[32], motor_percent[8], wifi_rssi |
| `Pid`         | 0x03 |   1 Hz   | `SpiPayloadPid` (108 B): 9 × PidAxis (kp, ki, kd) |
| `CalibStatus` | 0x04 | on event | `SpiPayloadCalibStatus` (67 B): target, status, progress, message[64] |
| `Log`         | 0x05 | on event | `SpiPayloadLog` (129 B): level, text[128] |

CRC covers `sizeof(SpiFrameHeader) + payload_len` bytes, using CRC-16/CCITT (poly 0x1021, init 0xFFFF).

#### 5.5.2 ESP32 → FC frame (MISO)

```
[magic 2B] [has_cmd 1B] [cmd_type 1B] [cmd payload 27B] [CRC16 2B] [pad 223B]
```

| Field | Type | Description |
|-------|------|-------------|
| `magic` | uint16 | `0xCAFE` |
| `has_cmd` | uint8 | `1` = a GCS command is present; FC ignores `cmd` if `0` |
| `cmd_type` | uint8 | `PKT_SET_PID (0x10)` or `PKT_CALIB_CMD (0x20)` |
| `cmd` | union[27B] | Raw packet bytes (header + payload + CRC) sized to the largest command |
| `crc` | uint16 | CRC-16/CCITT over `magic + has_cmd + cmd_type + cmd` |
| `pad` | uint8[223] | Zero-padding to reach 256 bytes |

The pending command is cleared from the MISO buffer immediately after being clocked out (i.e. on the same `buildTxFrame()` call that includes it).

---

### 5.6 MTF-01 MAVLink v1 Parser

#### 5.6.1 Wire format

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
6+LEN    2    CRC   — little-endian CRC16-X25 (see §5.6.3)
```

Total frame size: `LEN + 8` bytes.

#### 5.6.2 Relevant message types

Only two message IDs are parsed; all others are discarded.

---

**MSG ID 100 — OPTICAL_FLOW** (payload = 26 bytes)

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

#### 5.6.3 CRC16-X25 calculation

The checksum covers bytes `[LEN, SEQ, SYSID, COMPID, MSGID, PAYLOAD...]` (everything after STX, excluding the CRC bytes themselves), followed by one **message-specific extra byte**:

```
crc = 0xFFFF
for each byte b in [LEN..PAYLOAD]:
    crc = crc16_x25_update(crc, b)
crc = crc16_x25_update(crc, extra_byte)   // 175 for MSG 100, 85 for MSG 132
```

CRC16-X25 polynomial: `0x1021`, reflected (LSB-first), init `0xFFFF`.
The two CRC bytes in the frame are stored **little-endian** (CRC_L first, CRC_H second).

#### 5.6.4 Parser state machine

The parser processes incoming bytes one at a time using an 8-state machine:

```
WAIT_STX → LEN → SEQ → SYSID → COMPID → MSGID → PAYLOAD → CRC_L → CRC_H
              ↑                                                         |
              └──────────────── bad CRC or unknown MSGID ──────────────┘
                                (reset to WAIT_STX)
```

| State      | Action |
|------------|--------|
| `WAIT_STX` | Wait for byte `0xFE`; advance to `LEN` |
| `LEN`      | Store payload length; reset CRC accumulator and feed `LEN` into it; advance to `SEQ` |
| `SEQ`      | Store seq; feed into CRC; advance to `SYSID` |
| `SYSID`    | Store sysid; feed into CRC; advance to `COMPID` |
| `COMPID`   | Store compid; feed into CRC; advance to `MSGID` |
| `MSGID`    | Store msgid; feed into CRC; if msgid ∉ {100, 132}: reset to `WAIT_STX`; else advance to `PAYLOAD` |
| `PAYLOAD`  | Accumulate bytes into a fixed 26-byte buffer; feed into CRC; after `LEN` bytes advance to `CRC_L` |
| `CRC_L`    | Store low CRC byte; advance to `CRC_H` |
| `CRC_H`    | Store high CRC byte; feed extra byte into CRC; compare with accumulated CRC; on match dispatch message; reset to `WAIT_STX` |

Payload buffer is always 26 bytes (size of the largest message). Bytes beyond the declared `LEN` are never read.

#### 5.6.5 Output and rate

On each valid frame:
- **MSG 100**: update `flow_x`, `flow_y`, `quality` in the latest `Mtf01Data` snapshot
- **MSG 132**: update `distance_m` in the latest `Mtf01Data` snapshot
- After either update, set a `newData` flag

The MTF-01 sends both messages at approximately 50 Hz each. The `Mtf01Reader::update()` method drains the UART ring buffer and calls the state machine for every available byte. The GCS packet `PKT_MTF01 (0x03)` is built and sent whenever `newData` is true.

#### 5.6.6 Interface (`Mtf01Reader` class)

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

WiFi credentials are stored in `Secrets.h` (not versioned — see §4.4). All other parameters are compile-time constants in the sketch for now; NVS-based runtime configuration is a future improvement.

| Parameter | Current approach | Future |
|-----------|-----------------|--------|
| WiFi SSID / password | `Secrets.h` (compile-time) | NVS |
| GCS IP | Subnet broadcast (automatic) | NVS |
| GCS port | Hardcoded `5005` | NVS |
| GPS baud rate | Hardcoded | NVS |
| Barometer I2C address | Hardcoded `0x77` (BMP180) | NVS |

---

## 7. Non-Functional Requirements

- All sensor reads and SPI transactions must be **non-blocking** — `loop()` must never block
- SPI slave initialisation failure is **non-fatal**: the firmware continues and sends UDP heartbeats
- Serial monitor (USB UART) outputs human-readable status at 1 Hz for debugging
- Firmware built with **Arduino Core for ESP32**, C++17, bare metal (no FreeRTOS tasks)
- Brace style: **Allman** — see `CODING_STYLE.md` at the repository root

---
