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
py simulator.py
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

## Rules for code modifications

- When modifying code, **propose updating `requirements/REQUIREMENTS.md`** if the spec changes.
- When modifying the UDP protocol or telemetry data, **propose updating `simulator.py`** to stay consistent.
- Always rebuild after changes and fix all errors before committing.
- Commit and push to `main` after each complete feature.
