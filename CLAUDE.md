# CLAUDE.md — Ground Control Station

## Project

Desktop GCS for a custom quadcopter/octocopter. Bidirectional WiFi UDP communication at 100 Hz.

## Stack

- **Language**: C++17, **UI**: Qt6 Widgets, **3D**: OpenGL (QOpenGLWidget), **Build**: CMake, **Network**: QUdpSocket
- **Simulator**: `simulator.py` (Python 3) — sends realistic UDP packets to `localhost:5005`

## Build

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="" -DCMAKE_PREFIX_PATH="C:/Qt/6.8.2/msvc2022_64"
cmake --build build --config Release --parallel
```

## Run

```bash
start build/Release/GroundControlStation.exe
start "GCS Simulator" cmd /k "py simulator.py"
```

## Architecture

The project follows a **strict MVC separation**:
- **Backend** (`src/backend/`) — all business logic: UDP protocol, packet parsing, telemetry state, command sending. **Must have zero Qt Widget dependency.** This layer owns the data.
- **UI** (`src/ui/`) — views only. Widgets read from the backend state and send user commands through a defined interface. No business logic here.
- Data flows from backend to UI exclusively via **Qt signals/slots** (`Qt::QueuedConnection` across threads).
- New features must respect this boundary — never put network or data logic inside a widget.

- Threading: UI on main thread, network on a dedicated `QThread`.
- `requirements/REQUIREMENTS.md` — full protocol and widget specification.

## UDP Protocol

All packets: 12-byte header (`0xABCD`, version, type, timestamp, seq, payload_len) + CRC-16/CCITT.
Full struct definitions in `src/backend/Protocol.h` and `requirements/REQUIREMENTS.md`.

## Comments

The code must be well commented. Add comments freely wherever they bring value: non-obvious logic, protocol details, OpenGL math, thread-safety invariants, design decisions. The only exception is when the code is already fully self-explanatory through good naming — in that case, a comment that merely restates what the code does adds no value and should be omitted. When in doubt, prefer to comment.

## Language

All code, comments, commit messages, documentation, and README files must be written in **English**. No exceptions.

## Logging

The application writes one log file per session to `<exe_dir>/logs/gcs_YYYY-MM-DD_HH-mm-ss.log`. The directory is created automatically. When more than 10 files exist, the oldest is deleted. Files are written via `QFile::write()` + `flush()` so data survives crashes.

**Log these events:**
- App startup / shutdown
- Network connect / disconnect (with drone IP)
- Camera open / close / errors
- Packet parser errors (bad magic, CRC failure, unknown packet type)
- PID command sent (axis + values)
- Any unrecoverable error or unexpected state

**Do NOT log:**
- Per-packet telemetry (100 Hz — would fill disk immediately)
- Normal UI interactions
- Map tile fetches

**Format:** `[YYYY-MM-DD HH:mm:ss.zzz] [LEVEL] message` — levels: `INFO`, `WARN`, `ERROR`.

When adding a new feature or subsystem, add the relevant log calls (start, stop, errors). The goal is that if the app crashes silently, the log file gives enough context to understand what happened.

## Unit tests

The test suite lives in `tests/` and is built as the `GCSTests` target.

**Any code modification that adds or changes backend logic must be accompanied by unit tests.** This is mandatory, not optional:
- New packet type → add decode tests in `TestPacketParser.cpp`
- New command → add structure + retry tests in `TestCommandSender.cpp`
- New backend class → add a new `TestFoo.cpp`, register its runner in `main.cpp` and `tests/CMakeLists.txt`
- Bug fix → add a regression test that would have caught the bug

Run the tests after every change:
```powershell
$env:PATH = "C:/Qt/6.8.2/msvc2022_64/bin;$env:PATH"
cmake --build build --target GCSTests
build\tests\Release\GCSTests.exe
```
All tests must pass before committing.

## Rules for code modifications

- When modifying code, **propose updating `requirements/REQUIREMENTS.md`** if the spec changes.
- When modifying the UDP protocol or telemetry data, **propose updating `simulator.py`** to stay consistent.
- Always rebuild after changes and fix all errors before committing.
- Commit and push to `main` after each complete feature.
