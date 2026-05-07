# Ground Control Station — Architecture Document

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Technology Stack](#2-technology-stack)
3. [Repository Structure](#3-repository-structure)
4. [Layered Architecture](#4-layered-architecture)
5. [Threading Model](#5-threading-model)
6. [Data Flow](#6-data-flow)
7. [Backend Layer](#7-backend-layer)
   - 7.1 [Protocol](#71-protocol)
   - 7.2 [AppLogger](#72-applogger)
   - 7.3 [UdpLink](#73-udplink)
   - 7.4 [PacketParser](#74-packetparser)
   - 7.5 [TelemetryState](#75-telemetrystate)
   - 7.6 [CommandSender](#76-commandsender)
8. [UI Layer](#8-ui-layer)
   - 8.1 [MainWindow](#81-mainwindow)
   - 8.2 [Dashboard Tab — Widget Grid](#82-dashboard-tab--widget-grid)
   - 8.3 [Graph Tab](#83-graph-tab)
   - 8.4 [Map Tab](#84-map-tab)
   - 8.5 [Video Tab](#85-video-tab)
9. [Widget Catalogue](#9-widget-catalogue)
10. [Signal & Slot Wiring](#10-signal--slot-wiring)
11. [UDP Protocol Reference](#11-udp-protocol-reference)
12. [Simulator](#12-simulator)
13. [Build System](#13-build-system)
14. [Logging](#14-logging)
15. [Design Decisions & Known Constraints](#15-design-decisions--known-constraints)

---

## 1. Project Overview

A desktop **Ground Control Station (GCS)** for a custom quadcopter or octocopter drone. The GCS and the drone communicate over **WiFi UDP at 100 Hz**. Communication is **bidirectional**: the drone streams telemetry, the GCS can send configuration commands (PID tuning).

The application runs on **Windows** (primary target) and is designed to remain compatible with Linux.

---

## 2. Technology Stack

| Concern             | Technology                                  |
|---------------------|---------------------------------------------|
| Language            | C++17                                       |
| UI framework        | Qt6 Widgets                                 |
| 3D rendering        | OpenGL via `QOpenGLWidget` + `QOpenGLFunctions` |
| Network             | `QUdpSocket` (Qt6 Network)                  |
| Map tiles           | HTTP via `QNetworkAccessManager` + `QNetworkDiskCache` |
| Video capture       | Qt6 Multimedia (`QCamera`, `QMediaCaptureSession`, `QVideoSink`, `QVideoWidget`) |
| Build system        | CMake 3.16+, MSVC 2022 (Windows)            |
| Qt version          | 6.8.2 msvc2022\_64                          |
| Python simulator    | Python 3, `socket` stdlib                   |

---

## 3. Repository Structure

```
Ground_Control_Station/
├── CMakeLists.txt
├── CLAUDE.md                  # AI assistant rules and project conventions
├── ARCHITECTURE.md            # This document
├── requirements/
│   └── REQUIREMENTS.md        # Full feature specification
├── simulator.py               # Python UDP simulator (dev tool)
└── src/
    ├── main.cpp
    ├── backend/               # Pure business logic — zero Qt Widget dependency
    │   ├── AppLogger.h/.cpp   # Thread-safe file logger
    │   ├── Protocol.h         # All packet structs (#pragma pack)
    │   ├── TelemetryState.h/.cpp  # Mutex-protected drone state snapshot
    │   ├── UdpLink.h/.cpp     # UDP socket management + connection detection
    │   ├── PacketParser.h/.cpp    # Binary datagram parsing + CRC verification
    │   └── CommandSender.h/.cpp   # Outgoing packet builder + ACK retry
    └── ui/
        ├── MainWindow.h/.cpp  # Single application window; owns all objects
        └── widgets/           # One file pair per widget; views only
            ├── DroneWidget3D.h/.cpp
            ├── CompassWidget.h/.cpp
            ├── JoystickWidget.h/.cpp
            ├── Mtf01Widget.h/.cpp
            ├── GpsWidget.h/.cpp
            ├── MotorWidget.h/.cpp
            ├── StatusWidget.h/.cpp
            ├── PidConfigWidget.h/.cpp
            ├── GraphWidget.h/.cpp
            ├── TerminalWidget.h/.cpp
            ├── MapWidget.h/.cpp
            └── VideoWidget.h/.cpp
```

---

## 4. Layered Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                          UI LAYER                               │
│  MainWindow                                                     │
│  ├── Dashboard tab  (DroneWidget3D, Compass, Joystick, GPS,    │
│  │                   MTF-01, Motor, Status, PidConfig,Terminal) │
│  ├── Graph tab      (GraphWidget + toolbar)                     │
│  ├── Map tab        (MapWidget + PiP overlay)                   │
│  └── Video tab      (VideoWidget)                               │
│                                                                 │
│  Widgets are pure views. They receive data via Qt signals       │
│  and emit signals to request commands. No network code here.    │
├─────────────────────────────────────────────────────────────────┤
│                      BACKEND LAYER                              │
│  UdpLink ──► PacketParser ──► signals ──► MainWindow slots     │
│  CommandSender ◄── MainWindow ◄── PidConfigWidget signal       │
│  TelemetryState  (thread-safe state cache, used for CSV export) │
│  AppLogger       (persistent log file, called from both layers) │
└─────────────────────────────────────────────────────────────────┘
         ▲ UDP datagrams            ▼ UDP datagrams
┌─────────────────────────────────────────────────────────────────┐
│                    DRONE / SIMULATOR                            │
│  Real drone over WiFi  OR  simulator.py on localhost:5005       │
└─────────────────────────────────────────────────────────────────┘
```

**Strict rule**: The backend layer (`src/backend/`) must never include any Qt Widget header. It may use Qt Core and Qt Network only. The UI layer may include anything.

---

## 5. Threading Model

```
Main Thread (Qt UI thread)
│
│  All widgets, MainWindow, status bar timer, map tile callbacks
│
├── QThread: Network thread
│   │
│   ├── UdpLink        (QUdpSocket::readyRead, timeout timer)
│   ├── PacketParser   (called synchronously from UdpLink::onReadyRead)
│   └── CommandSender  (retry timer, sendDatagram)
│
└── QNetworkAccessManager (HTTP tile fetches for MapWidget)
    Callbacks run on the main thread via Qt's event loop.
```

**Cross-thread communication**: all signals from `UdpLink` and `PacketParser` to `MainWindow` slots are connected with `Qt::QueuedConnection`. This serialises the cross-thread calls safely through the Qt event queue.

**CommandSender invocation**: `PidConfigWidget` emits `sendPidRequested`. `MainWindow` captures this and dispatches via `QMetaObject::invokeMethod(..., Qt::QueuedConnection)` so `sendSetPid()` always executes on the network thread, where the socket lives.

**TelemetryState**: protected by `QMutex`. Written by the network thread via `PacketParser` signals → `MainWindow` slots → `m_state.updateXxx()`. Read by the UI (e.g., `exportCsv` reads `m_state.pid()`).

**AppLogger**: protected by `QMutex`. Can be called from any thread without additional synchronisation.

---

## 6. Data Flow

### Incoming telemetry (Drone → GCS)

```
Drone UDP packet
  └─► QUdpSocket::readyRead
        └─► UdpLink::onReadyRead()
              ├── update drone endpoint / connection state
              └─► PacketParser::parse(datagram)
                    ├── CRC-16/CCITT verification
                    ├── dispatch by type
                    └─► emit attitudeReceived / gpsReceived / ... (QueuedConnection)
                          └─► MainWindow::onXxxReceived(data)
                                ├── m_state.updateXxx(data)   // TelemetryState
                                └── widget->updateData(data)  // UI update
```

### Outgoing command (GCS → Drone)

```
User edits PID fields in PidConfigWidget and clicks "Send"
  └─► PidConfigWidget::sendPidRequested(axis, kp, ki, kd)  [signal]
        └─► MainWindow lambda (QueuedConnection)
              └─► QMetaObject::invokeMethod(m_cmdSender, [axis,kp,ki,kd], QueuedConnection)
                    └─► CommandSender::sendSetPid(axis, kp, ki, kd)  [network thread]
                          ├── build PktSetPid with CRC
                          ├── UdpLink::sendDatagram(data)
                          └── start retry timer (3 retries × 200 ms, waits for PKT_ACK)
```

### Video frame flow (Camera → VideoWidget + Map PiP)

```
QCamera
  └─► QMediaCaptureSession
        └─► QVideoSink (m_sink)  ← sole session output
              └─► videoFrameChanged signal
                    ├─► VideoWidget's QVideoWidget::videoSink()::setVideoFrame()
                    └─► MapWidget's PiP QVideoWidget::videoSink()::setVideoFrame()
```

This avoids opening the camera device twice. `QMediaCaptureSession::addVideoOutput()` does not exist in Qt 6.8; the `QVideoSink` fan-out pattern is used instead.

---

## 7. Backend Layer

### 7.1 Protocol

**File**: `src/backend/Protocol.h`

All structures are packed (`#pragma pack(1)`). Every packet starts with a 12-byte `PacketHeader` and ends with a 2-byte CRC-16/CCITT.

```
┌──────────┬─────────┬──────┬──────────────┬───────┬─────────────┐
│ magic    │ version │ type │ timestamp_us │  seq  │ payload_len │
│ 0xABCD   │  0x01   │ u8   │    u32       │  u16  │    u16      │
│  2 bytes │  1 byte │1 byte│   4 bytes    │2 bytes│   2 bytes   │
└──────────┴─────────┴──────┴──────────────┴───────┴─────────────┘
[  payload bytes (payload_len)  ] [ CRC u16 ]
```

**Drone → GCS packet types:**

| ID     | Name           | Key payload fields                                          |
|--------|----------------|-------------------------------------------------------------|
| `0x01` | `PKT_ATTITUDE` | qw/qx/qy/qz quaternion, gx/gy/gz (°/s), ax/ay/az (m/s²)  |
| `0x02` | `PKT_GPS`      | lat/lon (double), altitude, speed, heading, satellites, fix_type |
| `0x03` | `PKT_MTF01`    | distance_m, flow_x/y (px/s), quality                       |
| `0x04` | `PKT_RADIO`    | 8×uint16 RC channels (1000–2000 µs), rssi                  |
| `0x05` | `PKT_STATUS`   | battery_voltage/%, FSM state string, 8×motor_%, wifi_rssi  |
| `0x06` | `PKT_PID`      | 9 axes × (kp, ki, kd): Rate·{Roll,Pitch,Yaw}, Att·{…}, Pos·{…} |
| `0x07` | `PKT_LOG`      | level (0–3), text[128] UTF-8                                |

**GCS → Drone:**

| ID     | Name          | Payload                                     |
|--------|---------------|---------------------------------------------|
| `0x10` | `PKT_SET_PID` | axis_id (PidAxisId enum), kp, ki, kd (float)|

**Drone → GCS (ACK):**

| ID     | Name      | Payload                              |
|--------|-----------|--------------------------------------|
| `0x11` | `PKT_ACK` | ack_type, ack_seq, success (0/1)     |

**CRC**: CRC-16/CCITT, polynomial 0x1021, init 0xFFFF. Covers header + payload (not the CRC field itself). Computed identically in `PacketParser::crc16()` and `CommandSender::crc16()`.

---

### 7.2 AppLogger

**Files**: `src/backend/AppLogger.h` / `.cpp`

Static class (no instance). All methods are thread-safe via a file-static `QMutex`.

```
AppLogger::init()    — opens <exe_dir>/logs/gcs_YYYY-MM-DD_HH-mm-ss.log
                       deletes oldest file if > 10 files in logs/
AppLogger::info(msg) — writes [timestamp] [INFO] msg\n
AppLogger::warn(msg) — writes [timestamp] [WARN] msg\n
AppLogger::error(msg)— writes [timestamp] [ERROR] msg\n
AppLogger::close()   — flushes and closes
```

Each write calls `QFile::write()` (no QTextStream intermediate buffer) then `QFile::flush()` to push to the OS immediately — data survives crashes.

**What is logged**:
- App startup / shutdown
- UDP bind, drone connect/disconnect/endpoint change
- Camera open/stop/errors, camera count
- PacketParser: bad magic (throttled 1/s), CRC mismatch, unknown type
- CommandSender: PID command sent (axis + values), retry exhausted

---

### 7.3 UdpLink

**Files**: `src/backend/UdpLink.h` / `.cpp`

Lives on the network `QThread`. Started by `MainWindow` which calls `m_netThread->start()`, triggering `UdpLink::start()` via `QThread::started`.

**Responsibilities**:
- Bind `QUdpSocket` on the configured port (default: 5005)
- Read all pending datagrams on `readyRead`; forward each to `PacketParser::parse()`
- Track the drone's source `IP:port` — emit `droneEndpointUpdated` on change
- Emit `connectionStateChanged(true)` on first packet; `connectionStateChanged(false)` after 2 s silence
- Provide `sendDatagram(QByteArray)` for outgoing packets
- Provide `lastPacketAgeMs()` for the status bar latency display

**Connection detection**: a 500 ms `QTimer` (`onTimeoutCheck`) compares `QDateTime::currentMSecsSinceEpoch()` against the last received packet timestamp.

---

### 7.4 PacketParser

**Files**: `src/backend/PacketParser.h` / `.cpp`

Called synchronously from `UdpLink::onReadyRead()` on the network thread.

**Parsing loop** (`parse()` → `tryParseOne()`):
1. Check minimum header size
2. Verify magic (`0xABCD`) and version (`0x01`) — bad bytes skipped one at a time (resync); logged at most once/second via `QElapsedTimer` throttle
3. Compute expected total size = `sizeof(PacketHeader) + payload_len + 2`
4. Verify CRC-16/CCITT — mismatch logged and packet skipped
5. Track sequence numbers for per-type packet loss computation
6. Dispatch by type: populate the matching `XxxData` struct and emit the corresponding signal

**Packet loss** (`packetLoss(type)`): maintains `SeqTracker` per type (received / expected counters). Used by the status bar to show `%` loss for `PKT_ATTITUDE`.

---

### 7.5 TelemetryState

**Files**: `src/backend/TelemetryState.h` / `.cpp`

Plain data container protected by `QMutex`. Acts as a **last-known-good snapshot** of the drone state, updated by `MainWindow` slots on the UI thread (after `QueuedConnection` delivery from the network thread), and read by the UI on demand.

Currently used only for `m_state.pid()` in the `exportCsv` path (to write PID values in the CSV header). All real-time widget updates go directly from the signal to the widget without reading `TelemetryState`.

**Data structs** (all in `TelemetryState.h`):

| Struct          | Fields                                                     |
|-----------------|------------------------------------------------------------|
| `AttitudeData`  | qw/qx/qy/qz, gx/gy/gz, ax/ay/az                          |
| `GpsData`       | latitude, longitude, altitude_m, speed_ms, heading_deg, satellites, fix_type |
| `Mtf01Data`     | distance_m, flow_x, flow_y, quality                        |
| `RadioData`     | channels[8], rssi                                          |
| `StatusData`    | battery_voltage, battery_percent, state (string), motor_percent[8], wifi_rssi, uptime_us |
| `PidData`       | 9 × `PidAxis{kp,ki,kd}` (rate/attitude/position × 3 axes) |

---

### 7.6 CommandSender

**Files**: `src/backend/CommandSender.h` / `.cpp`

Lives on the network thread. Handles **reliable delivery** of outgoing commands via an ACK/retry loop.

**`sendSetPid(axis, kp, ki, kd)`**:
1. Assigns a monotonically incrementing sequence number
2. Builds a `PktSetPid` packet with CRC
3. Stores a `PendingCmd` in `m_pending` map (seq → cmd)
4. Sends via `UdpLink::sendDatagram()`
5. Starts the retry timer (200 ms interval)

**Retry loop** (`onRetryTimer`): for each pending command, decrements `retriesLeft` and retransmits. After 3 failed retries the command is dropped and a warning is logged.

**ACK handling** (`onAckReceived`): removes the matching seq from `m_pending`, stopping retries.

---

## 8. UI Layer

### 8.1 MainWindow

**Files**: `src/ui/MainWindow.h` / `.cpp`

The single application window. Owns every object in the process: the network thread, both backend objects, all widgets, the status bar labels, and the status timer.

**Lifetime responsibilities**:
```
Constructor:
  setupUi()          — create all widgets, tabs, toolbar buttons, wire PiP
  setupStatusBar()   — add labels to the Qt status bar
  connectSignals()   — wire all backend signals → MainWindow slots
  Create QThread + UdpLink + CommandSender
  Start network thread

Destructor:
  Quit + wait network thread
  Delete UdpLink, CommandSender
```

**Status bar** (always visible, outside tabs):

| Label             | Content                              |
|-------------------|--------------------------------------|
| `m_statusConn`    | "Connected" (green) / "Disconnected" (red) |
| `m_statusLatency` | Last packet age in ms                |
| `m_statusRssi`    | WiFi RSSI from `PKT_STATUS`          |
| `m_statusLoss`    | Packet loss % for `PKT_ATTITUDE`     |
| `m_statusPort`    | Fixed: "Port: 5005"                  |
| `m_statusDroneIp` | Last known source IP                 |

A 500 ms `QTimer` (`m_statusTimer`) calls `onStatusBarTick()` to refresh latency and loss.

---

### 8.2 Dashboard Tab — Widget Grid

```
Col:    0             1              2              3
Row 0: [DroneWidget3D][CompassWidget][JoystickWidget][MotorWidget ]
Row 1: [StatusWidget ][Mtf01Widget  ][GpsWidget     ][MotorWidget ] (spans rows 0-1)
Row 2: [TerminalWidget     ×2       ][PidConfigWidget      ×2    ]
```

Column stretches: all equal (2). Row stretches: 3 / 2 / 4.

---

### 8.3 Graph Tab

Full-area `GraphWidget` with a toolbar above it:

```
[Pause] [Play] [Export CSV] [Screenshot]      ← QPushButton toolbar
[                GraphWidget                 ]
[               scrollbar (paused only)      ]
```

`exportCsv` passes `m_state.pid()` from `MainWindow` to `GraphWidget` so PID values appear as comment lines at the top of the CSV.

---

### 8.4 Map Tab

Full-area `MapWidget` with absolutely positioned overlay widgets:

```
Top-right stack (8 px margin, 36 px steps):
  [⊕ Follow]
  [+] [−]
  [Trail ☑]
  [Satellite / Street map]
  [Video]          ← PiP toggle (blue = on)

Bottom-left:
  ┌────────────────────┐
  │  320×180 PiP video │  ← QVideoWidget inside QFrame
  └────────────────────┘

Bottom strip: attribution text (OSM or Esri)
Top-left HUD: "Zoom 15  · follow"
```

**Tile rendering**: Web Mercator projection (EPSG:3857). World-pixel coordinate system: origin at northwest corner, `TILE_SIZE=256` px per tile at zoom 0. At zoom `z`: `2^z × 2^z` tiles. Drone and trail coordinates converted to widget-space via:
```
widget_x = world_x - m_centerPixX + width/2
widget_y = world_y - m_centerPixY + height/2
```

---

### 8.5 Video Tab

Full-area `VideoWidget`:

```
[Camera input:] [Integrated Camera ▼] [Refresh] [status label]
[                  QVideoWidget (live feed)                   ]
```

Camera session is started immediately when a device is selected. The session's output is a `QVideoSink` (`m_sink`). Frames from `m_sink` are pushed to both the `QVideoWidget` in this tab and the PiP overlay in the Map tab.

---

## 9. Widget Catalogue

| Widget           | Class              | Data received             | Signal emitted              |
|------------------|--------------------|---------------------------|-----------------------------|
| 3D Drone View    | `DroneWidget3D`    | `AttitudeData` (quaternion)| —                          |
| Compass          | `CompassWidget`    | heading from `GpsData`    | —                           |
| Joystick         | `JoystickWidget`   | `RadioData` (8 channels)  | —                           |
| MTF-01           | `Mtf01Widget`      | `Mtf01Data`               | —                           |
| GPS              | `GpsWidget`        | `GpsData`                 | —                           |
| Motors           | `MotorWidget`      | `StatusData` (motor_%)    | —                           |
| Status           | `StatusWidget`     | `StatusData`              | —                           |
| PID Config       | `PidConfigWidget`  | `PidData` (on PKT_PID)    | `sendPidRequested(axis,kp,ki,kd)` |
| Graph            | `GraphWidget`      | `AttitudeData`, `GpsData`, `Mtf01Data` | — |
| Terminal         | `TerminalWidget`   | level + text (PKT_LOG)    | —                           |
| Map              | `MapWidget`        | `GpsData`, `QVideoSink*`  | —                           |
| Video            | `VideoWidget`      | OS camera device          | —                           |

**DroneWidget3D** (`QOpenGLWidget`): renders 4 arms + rotors using fixed-function OpenGL (`glBegin/glEnd`). Quaternion → 4×4 column-major rotation matrix applied via `glMultMatrixf`. Requires `opengl32.lib` on Windows.

**GraphWidget**: 8 curves, 10 s rolling window. Two parallel deque arrays: `m_curves` (trimmed to window for display) and `m_history` (full session, never trimmed, used for CSV and paused scrollback). Auto-scaling Y axis. Pause mode: amber border, horizontal scrollbar with 0.1 s resolution; `paintEvent` uses binary search on `m_history` to find the visible range.

**MapWidget**: tile cache is a `QMap<QString, QPixmap>` keyed as `"layer/z/x/y"` (layer = `"osm"` or `"sat"`). In-flight requests tracked as `QMap<QString, QNetworkReply*>` (`m_pendingReplies`); all aborted on zoom/layer change via `cancelPendingRequests()` to free HTTP slots immediately (fix for the black-tile bug).

**VideoWidget**: camera list populated from `QMediaDevices::videoInputs()`. Bug: when items are added to the QComboBox while signals are blocked, Qt silently sets `currentIndex=0`; a subsequent `setCurrentIndex(0)` emits no signal — `startCamera()` must be called explicitly after repopulating.

---

## 10. Signal & Slot Wiring

All cross-thread connections use `Qt::QueuedConnection`.

```
UdpLink::connectionStateChanged(bool)
  → MainWindow::onConnectionChanged(bool)        → status bar label

UdpLink::droneEndpointUpdated(QString, quint16)
  → MainWindow::onDroneEndpointUpdated(...)      → status bar label

PacketParser::attitudeReceived(AttitudeData)
  → MainWindow::onAttitudeReceived(AttitudeData)
      → m_state.updateAttitude()
      → m_drone3d->updateAttitude()
      → m_graph->pushAttitude()

PacketParser::gpsReceived(GpsData)
  → MainWindow::onGpsReceived(GpsData)
      → m_state.updateGps()
      → m_gps->updateData()
      → m_compass->setHeading()
      → m_graph->pushGps()
      → m_map->updatePosition()

PacketParser::mtf01Received(Mtf01Data)
  → MainWindow::onMtf01Received(Mtf01Data)
      → m_state.updateMtf01()
      → m_mtf01->updateData()
      → m_graph->pushMtf01()

PacketParser::radioReceived(RadioData)
  → MainWindow::onRadioReceived(RadioData)
      → m_state.updateRadio()
      → m_joystick->updateData()

PacketParser::statusReceived(StatusData)
  → MainWindow::onStatusReceived(StatusData)
      → m_state.updateStatus()
      → m_motor->updateData()
      → m_status->updateData()
      → status bar RSSI label

PacketParser::pidReceived(PidData)
  → MainWindow::onPidReceived(PidData)
      → m_state.updatePid()
      → m_pid->updatePid()

PacketParser::logReceived(uint8_t, QString)
  → MainWindow::onLogReceived(...)
      → m_terminal->appendMessage()

PidConfigWidget::sendPidRequested(PidAxisId, float, float, float)
  → MainWindow lambda (direct, UI thread)
      → QMetaObject::invokeMethod(m_cmdSender, QueuedConnection)
          → CommandSender::sendSetPid()           [network thread]

QVideoSink::videoFrameChanged(QVideoFrame)          [VideoWidget::m_sink]
  → VideoWidget's QVideoWidget::videoSink()::setVideoFrame()
  → MapWidget's PiP QVideoWidget::videoSink()::setVideoFrame()
```

---

## 11. UDP Protocol Reference

### Packet structure (all fields little-endian)

```
Offset  Size  Field
0       2     magic         = 0xABCD
2       1     version       = 0x01
3       1     type          (see table)
4       4     timestamp_us  (µs since drone boot)
8       2     seq           (per-type counter)
10      2     payload_len   (bytes, excludes header and CRC)
12      N     payload       (N = payload_len)
12+N    2     CRC-16/CCITT  (poly 0x1021, init 0xFFFF, covers bytes 0..11+N)
```

### CRC algorithm

```cpp
uint16_t crc = 0xFFFF;
for each byte b:
    crc ^= (uint16_t)b << 8;
    for 8 bits:
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
```

### Connection lifecycle

- GCS listens on `0.0.0.0:5005`
- On first received datagram: `connectionStateChanged(true)`, log drone IP
- On 2 s silence: `connectionStateChanged(false)`, log timeout
- GCS sends commands to the last known drone `IP:port`

---

## 12. Simulator

**File**: `simulator.py`

Python 3 script. Sends realistic UDP telemetry to `127.0.0.1:5005` at ~100 Hz. Simulates:
- Sinusoidal attitude (roll/pitch oscillation, slow yaw rotation)
- GPS coordinates near a fixed base position with random walk
- MTF-01 distance and optical flow
- RC radio channels
- Battery discharge over time
- FSM states (IDLE → ARMED → FLYING)
- PID values broadcast on `PKT_PID`
- Log messages at various levels
- Packet sequence numbers (per type) for loss detection

Used exclusively for development — not part of the production build.

**Run**: `py simulator.py` (Windows) or `python3 simulator.py` (Linux)

---

## 13. Build System

**File**: `CMakeLists.txt`

```cmake
find_package(Qt6 REQUIRED COMPONENTS
    Core Gui Widgets Network OpenGL OpenGLWidgets
    Multimedia MultimediaWidgets)
```

`CMAKE_AUTOMOC ON` handles Qt's meta-object compiler automatically.

**Build commands** (from repo root):
```bash
# Configure
cmake -B build -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_PREFIX_PATH="C:/Qt/6.8.2/msvc2022_64"

# Compile
cmake --build build --config Release --parallel
```

**Windows deployment** (first time, or after adding Qt modules):
```
windeployqt.exe build\Release\GroundControlStation.exe
```
Required for multimedia backend plugins (`ffmpegmediaplugin.dll`, `windowsmediaplugin.dll`, FFmpeg DLLs) without which `QMediaDevices::videoInputs()` returns an empty list.

**Output**: `build/Release/GroundControlStation.exe` (WIN32 subsystem, no console).

---

## 14. Logging

**Runtime log location**: `<exe_dir>/logs/gcs_YYYY-MM-DD_HH-mm-ss.log`

One file per application launch, named with the startup timestamp. If more than 10 log files exist, the oldest is deleted before creating the new one.

Each line format:
```
[2026-05-07 22:11:50.225] [INFO] VideoWidget: 1 camera(s) detected
[2026-05-07 22:11:50.274] [INFO] VideoWidget: camera started "Integrated Camera"
[2026-05-07 22:11:50.564] [INFO] UdpLink: listening on UDP port 5005
[2026-05-07 22:11:50.574] [INFO] UdpLink: drone endpoint updated to 127.0.0.1:63264
[2026-05-07 22:11:50.574] [INFO] UdpLink: connected to drone at 127.0.0.1:63264
```

Levels: `INFO`, `WARN`, `ERROR`. Written with `QFile::write()` + `flush()` after every line (crash-safe). Thread-safe via `QMutex`.

---

## 15. Design Decisions & Known Constraints

### MVC separation
The backend has **zero Qt Widget dependency**. This ensures it can be unit-tested without a display, ported to a headless system, or reused in a different UI framework. New features must respect this boundary.

### QVideoSink fan-out (video PiP)
`QMediaCaptureSession::addVideoOutput()` was added in Qt 6.7 but is absent from Qt 6.8.2 MSVC build. The workaround routes all frames through a single `QVideoSink` and uses `connect(sink, &QVideoSink::videoFrameChanged, widget->videoSink(), &QVideoSink::setVideoFrame)` for each consumer. This is functionally equivalent and has no measurable overhead.

### Map tile black-tile bug fix
Rapid zoom changes left old in-flight HTTP replies occupying all `MAX_INFLIGHT=6` slots, blocking new tiles from loading. Fix: replace the `QSet<QString> m_pending` (key-only) with `QMap<QString, QNetworkReply*> m_pendingReplies` (key → reply pointer), and call `reply->abort()` on all in-flight requests whenever zoom or layer changes (`cancelPendingRequests()`).

### QComboBox currentIndex signal not emitted
When items are added to a `QComboBox` while signals are blocked (`blockSignals(true)`), Qt internally sets `currentIndex=0` for the first item. A subsequent `setCurrentIndex(0)` does not emit `currentIndexChanged` because the index did not change. Fix: call `startCamera()` explicitly after `setCurrentIndex()` instead of relying on the signal.

### OpenGL on Windows
Fixed-function OpenGL (`glBegin/glEnd`) requires linking `opengl32.lib`. This is handled by `target_link_libraries(${PROJECT_NAME} PRIVATE opengl32)` in CMake, conditional on `WIN32`.

### Tile cache key namespacing
OSM and Esri satellite tiles use the same `z/x/y` coordinate space but different images. Without namespacing, switching layers could display tiles from the wrong layer out of the in-memory cache. Fix: prefix every cache key with `"osm/"` or `"sat/"`.

### Thread-safe state vs. direct widget update
`TelemetryState` is updated at every packet but widgets are updated directly from `MainWindow` slots (not by reading `TelemetryState`). `TelemetryState` exists primarily for on-demand reads from the UI thread (CSV export) and could be the source for future features that need snapshot access outside the signal chain.
