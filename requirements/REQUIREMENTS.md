# GCS — Ground Control Station
## Requirements Document for Claude Code

---

## 1. Overview

Desktop application acting as a Ground Control Station (GCS) for a custom quadcopter drone.
The drone communicates with the GCS over **WiFi UDP** at **100 Hz**.
Communication is **bidirectional**: the drone streams telemetry to the GCS, and the GCS can send configuration commands to the drone.

---

## 2. Tech Stack

| Component | Technology |
|-----------|------------|
| Language | C++17 or later |
| UI Framework | Qt6 (Widgets) |
| 3D Rendering | OpenGL (via QOpenGLWidget) |
| Build System | CMake |
| Network | QUdpSocket (Qt6 Network module) |

---

## 3. Architecture

### 3.1 General Principles

- **Strict separation between business logic and UI (MVC or similar pattern)**
- Network, data parsing, and state management must live in a backend layer with no Qt Widget dependency
- The UI layer only reads from the backend state and sends user commands through a defined interface
- Use Qt signals/slots to propagate data from backend to UI

### 3.2 Directory Structure (suggested)

```
gcs/
├── CMakeLists.txt
├── src/
│   ├── main.cpp
│   ├── backend/
│   │   ├── UdpLink.h / .cpp          # UDP socket management
│   │   ├── PacketParser.h / .cpp     # Binary packet parsing & CRC check
│   │   ├── TelemetryState.h / .cpp   # Current drone state (thread-safe)
│   │   └── CommandSender.h / .cpp    # Builds and sends command packets
│   └── ui/
│       ├── MainWindow.h / .cpp       # Single main window, layout management
│       ├── widgets/
│       │   ├── DroneWidget3D.h / .cpp
│       │   ├── CompassWidget.h / .cpp
│       │   ├── JoystickWidget.h / .cpp
│       │   ├── Mtf01Widget.h / .cpp
│       │   ├── GpsWidget.h / .cpp
│       │   ├── MotorWidget.h / .cpp
│       │   ├── PidConfigWidget.h / .cpp
│       │   ├── GraphWidget.h / .cpp
│       │   └── TerminalWidget.h / .cpp
```

### 3.3 Threading

- **Main thread**: Qt UI only
- **Network thread** (QThread or QUdpSocket with event loop): handles all UDP I/O
- Data shared between threads via **thread-safe queue or Qt signals** — no raw shared state

---

## 4. UDP Protocol

### 4.1 Common Packet Header

All packets (both directions) start with this header:

```cpp
struct PacketHeader {
    uint16_t magic;        // Always 0xABCD
    uint8_t  version;      // Protocol version = 0x01
    uint8_t  type;         // Packet type (see below)
    uint32_t timestamp_us; // Microseconds since drone boot
    uint16_t seq;          // Sequence number (increments per packet type)
    uint16_t payload_len;  // Byte length of payload (excluding header and CRC)
};
```

All packets end with:
```cpp
uint16_t crc; // CRC-16/CCITT over header + payload
```

### 4.2 Drone → GCS Packet Types

| Type ID | Name | Rate |
|---------|------|------|
| `0x01` | Attitude (quaternion) | 100 Hz |
| `0x02` | GPS | 10 Hz |
| `0x03` | MTF-01 (optical flow + laser) | 50 Hz |
| `0x04` | Radio receiver | 50 Hz |
| `0x05` | Status (battery, FSM state, motors) | 10 Hz |
| `0x06` | PID values | Streamed |
| `0x07` | Log / terminal text | On event |

#### 0x01 — Attitude
```cpp
struct PktAttitude {
    PacketHeader header;
    float qw, qx, qy, qz; // Unit quaternion
    float gx, gy, gz;      // Angular velocity °/s
    float ax, ay, az;      // Acceleration m/s²
    uint16_t crc;
};
```

#### 0x02 — GPS
```cpp
struct PktGps {
    PacketHeader header;
    double latitude;      // Decimal degrees
    double longitude;     // Decimal degrees
    float  altitude_m;    // Meters above sea level
    float  speed_ms;      // Ground speed m/s
    float  heading_deg;   // Course over ground, degrees
    uint8_t satellites;
    uint8_t fix_type;     // 0=none, 1=2D, 2=3D
    uint16_t crc;
};
```

#### 0x03 — MTF-01
```cpp
struct PktMtf01 {
    PacketHeader header;
    float distance_m;     // Laser rangefinder, meters
    float flow_x;         // Optical flow X, px/s
    float flow_y;         // Optical flow Y, px/s
    uint8_t quality;      // Flow quality 0–255
    uint16_t crc;
};
```

#### 0x04 — Radio Receiver
```cpp
struct PktRadio {
    PacketHeader header;
    uint16_t channels[8]; // Raw channel values (typically 1000–2000 µs)
    uint8_t  rssi;        // Receiver RSSI 0–255
    uint16_t crc;
};
```

#### 0x05 — Status
```cpp
struct PktStatus {
    PacketHeader header;
    float   battery_voltage;  // Volts
    float   battery_current;  // Amps
    uint8_t battery_percent;  // 0–100
    char    state[32];        // Null-terminated FSM state string (e.g. "IDLE", "ARMED", "FLYING")
    uint8_t motor_percent[8]; // Motor throttle 0–100 per motor (up to octocopter)
    uint8_t wifi_rssi;        // WiFi signal strength 0–100
    uint16_t crc;
};
```

#### 0x06 — PID Values
```cpp
struct PidAxis {
    float kp, ki, kd;
};

struct PktPidValues {
    PacketHeader header;
    PidAxis rate_roll;
    PidAxis rate_pitch;
    PidAxis rate_yaw;
    PidAxis attitude_roll;
    PidAxis attitude_pitch;
    PidAxis attitude_yaw;
    PidAxis position_x;
    PidAxis position_y;
    PidAxis position_z;
    uint16_t crc;
};
```

#### 0x07 — Log / Terminal
```cpp
struct PktLog {
    PacketHeader header;
    uint8_t  level;         // 0=DEBUG, 1=INFO, 2=WARN, 3=ERROR
    char     text[128];     // Null-terminated UTF-8 string
    uint16_t crc;
};
```

### 4.3 GCS → Drone Packet Types

| Type ID | Name |
|---------|------|
| `0x10` | Set PID coefficients |

#### 0x10 — Set PID
```cpp
struct PktSetPid {
    PacketHeader header;
    uint8_t axis_id;  // See axis enum below
    float   kp, ki, kd;
    uint16_t crc;
};

enum PidAxisId : uint8_t {
    RATE_ROLL = 0, RATE_PITCH, RATE_YAW,
    ATT_ROLL, ATT_PITCH, ATT_YAW,
    POS_X, POS_Y, POS_Z
};
```

### 4.4 ACK Mechanism

`0x10` (Set PID) requires an acknowledgement from the drone:

```cpp
struct PktAck {
    PacketHeader header;
    uint8_t  ack_type;  // Type of the packet being acknowledged
    uint16_t ack_seq;   // Sequence number of the acknowledged packet
    uint8_t  success;   // 1=ok, 0=error
    uint16_t crc;
};
```

- GCS retries a command up to **3 times** with **200 ms** timeout if no ACK is received
- Telemetry packets (0x01–0x07) do **not** require ACK

### 4.5 Passive Reception Model

The GCS is **purely passive** on the receiving side — it never requests data from the drone.
The drone streams all telemetry continuously. A single UDP datagram may contain **one or more packet types**, and not all packet types are guaranteed to be present in every datagram.
The GCS parses each received datagram and updates only the fields it finds.

### 4.5 Packet Loss Tracking

The GCS tracks sequence numbers per packet type to compute a **packet loss percentage** (displayed in the UI).

---

## 5. UI — Main Window

### 5.1 General Layout

- **Single window, no tabs, no menu bar**
- All widgets arranged in a responsive grid layout
- Dark theme (dark background, light text)
- Each widget has a visible title/label

### 5.2 Widget List

---

#### W1 — 3D Drone View (`DroneWidget3D`)

- Renders a simple 3D drone model (4 arms + rotors) using **QOpenGLWidget**
- Drone orientation is updated in real-time from the received quaternion (`PktAttitude`)
- Camera is fixed; only the drone model rotates
- Background: dark gray or black

---

#### W2 — Compass (`CompassWidget`)

- 2D circular compass widget
- Needle or rotating disc indicates current heading (derived from GPS course or yaw)
- Displays heading value in degrees as text

---

#### W3 — Radio / Joystick (`JoystickWidget`)

- Displays **2 virtual joysticks** side by side (Left: Throttle/Yaw, Right: Pitch/Roll)
- Joystick dots move in real-time based on received `PktRadio` channel values
- Shows raw channel values as numbers below each joystick (channels 1–8)
- Shows receiver RSSI value

---

#### W4 — MTF-01 Data (`Mtf01Widget`)

- Displays raw sensor values as labeled numbers:
  - `Distance`: X.XX m
  - `Flow X`: X.XX px/s
  - `Flow Y`: X.XX px/s
  - `Quality`: XXX / 255
- Values update at 50 Hz

---

#### W5 — GPS Data (`GpsWidget`)

- Displays raw GPS values as labeled numbers:
  - Latitude, Longitude (decimal degrees)
  - Altitude (m)
  - Speed (m/s)
  - Heading (°)
  - Satellites in view
  - Fix type (None / 2D / 3D) with color indicator (red/orange/green)

---

#### W6 — Motor Gauges (`MotorWidget`)

- **8 vertical gauge bars** labeled M1–M8, sized to fit in an **X layout**:
  - Top-left pair : M1, M2 (side by side)
  - Top-right pair : M3, M4 (side by side)
  - Bottom-left pair : M5, M6 (side by side)
  - Bottom-right pair : M7, M8 (side by side)
- A transparent center zone separates the four pairs, forming the X shape
- Each bar fills proportionally to `motor_percent[i]` (0–100%)
- Color: green at low throttle, yellow at mid, red near max
- Shows numeric percentage below each gauge
- Compatible with both quadcopter (M5–M8 at 0) and octocopter configurations

---

#### W7 — PID Configuration & Info (`PidConfigWidget`)

**Editable fields (send to drone):**

Three PID groups: **Rate**, **Attitude**, **Position**
Each group has Roll, Pitch, Yaw (or X, Y, Z for position) axes.
Each axis has **Kp, Ki, Kd** editable float fields.

A **"Send"** button per group sends the updated values to the drone via `PktSetPid`.
PID values displayed in the fields are updated automatically when `PktPidValues` is received from the drone.

**Read-only info fields:**
- Battery voltage (V)
- Battery level (%)
- Drone uptime (HH:MM:SS, derived from packet header `timestamp_us`)
- FSM state string received from the drone, displayed as a large color-coded label:
  - RED — state contains "DISARM" or "ERROR" or "FAULT"
  - GREEN — state contains "ARM", "FLY", or "LAND"
  - AMBER — any other state

---

#### W8 — Real-Time Graph (`GraphWidget`)

- Displays up to **8 simultaneous curves** in real-time
- Each curve has a **distinct color**
- Each curve can be **toggled on/off** via a labeled checkbox
- X-axis: rolling time window (e.g. last 10 seconds), configurable
- Y-axis: auto-scale or fixed range
- The 8 data sources are configurable (mapped to any numeric field from the telemetry state)
- Suggested default mapping:
  1. Roll (°)
  2. Pitch (°)
  3. Yaw (°)
  4. Altitude (m)
  5. Gyro X
  6. Gyro Y
  7. Gyro Z
  8. Flow quality

---

#### W9 — Terminal (`TerminalWidget`)

- Scrollable text area displaying log messages received via `PktLog`
- Each message prefixed with timestamp and log level
- Color-coded by level: DEBUG=gray, INFO=white, WARN=yellow, ERROR=red
- Auto-scrolls to latest message
- **Clear** button to reset content

---

### 5.3 Global Status Bar (top or bottom of window)

Always visible, outside any widget:

| Element | Description |
|---------|-------------|
| Connection indicator | Green dot = connected, Red = disconnected |
| UDP latency | Estimated round-trip or last packet age in ms |
| WiFi RSSI | Signal strength bar + numeric value |
| Packet loss | % of lost packets (per second) |
| Local UDP port | Port the GCS is listening on |
| Drone IP | Last known source IP |

---

## 6. Network Configuration

- GCS listens on a configurable UDP port (default: `5005`)
- Drone sends to the GCS IP and port (configured on the drone side)
- GCS sends commands back to the last known drone `IP:port`
- Connection is considered **lost** if no packet is received for **2 seconds** → UI shows disconnected state

---

## 7. Non-Functional Requirements

- Application must run on **Linux and Windows**
- UI must remain **responsive at 100 Hz** incoming data rate without frame drops
- All network I/O must be **non-blocking** relative to the UI thread
- Code must be **well-commented** (especially protocol parsing and OpenGL rendering)
- No external dependencies beyond Qt6 and standard OpenGL — no third-party UI libs

---

## 8. Out of Scope

- Flight logging to file (future feature)
