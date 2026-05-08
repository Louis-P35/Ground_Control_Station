"""
GCS Telemetry Simulator
Sends realistic fake drone telemetry to localhost:5005 via UDP.

Packet rates:
  0x01 Attitude  — 100 Hz
  0x02 GPS       —  10 Hz
  0x03 MTF-01    —  50 Hz
  0x04 Radio     —  50 Hz
  0x05 Status    —  10 Hz
  0x06 PID       —   1 Hz
  0x07 Log       — random events
  0x08 Baro      —  10 Hz
"""

import socket
import struct
import time
import math
import random

# ── Config ──────────────────────────────────────────────────────────────────
GCS_HOST = "127.0.0.1"
GCS_PORT = 5005

MAGIC   = 0xABCD
VERSION = 0x01

# ── CRC-16/CCITT ─────────────────────────────────────────────────────────────
def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc

# ── Packet builder ────────────────────────────────────────────────────────────
def make_packet(pkt_type: int, payload: bytes, seq: int, timestamp_us: int) -> bytes:
    header = struct.pack("<HBBIHH",
        MAGIC, VERSION, pkt_type,
        timestamp_us & 0xFFFFFFFF,
        seq & 0xFFFF,
        len(payload)
    )
    body = header + payload
    crc  = crc16(body)
    return body + struct.pack("<H", crc)

# ── Euler → quaternion ────────────────────────────────────────────────────────
def euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll/2),  math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2),   math.sin(yaw/2)
    return (
        cr*cp*cy + sr*sp*sy,   # qw
        sr*cp*cy - cr*sp*sy,   # qx
        cr*sp*cy + sr*cp*sy,   # qy
        cr*cp*sy - sr*sp*cy,   # qz
    )

# ── Simulation state ──────────────────────────────────────────────────────────
seqs = [0] * 0x20   # sequence counters per packet type

LOG_MESSAGES = [
    (1, "Stabilize mode active"),
    (1, "GPS fix acquired — 3D"),
    (1, "Battery OK: 12.4 V"),
    (2, "MTF-01 quality degraded"),
    (1, "PID tuning applied"),
    (3, "Motor 3 temperature high"),
    (1, "Arming complete"),
    (0, "Loop time: 2498 us"),
    (0, "Optical flow: OK"),
    (2, "WiFi RSSI dropping: 42"),
]

def next_seq(pkt_type: int) -> int:
    seqs[pkt_type] = (seqs[pkt_type] + 1) & 0xFFFF
    return seqs[pkt_type]

def simulate(sock, t: float):
    ts = int(t * 1_000_000) & 0xFFFFFFFF

    # ── 0x01 Attitude ────────────────────────────────────────────────
    roll  = math.radians(15 * math.sin(t * 0.7))
    pitch = math.radians(10 * math.sin(t * 0.5 + 1.0))
    yaw   = math.radians(45 * math.sin(t * 0.3))
    qw, qx, qy, qz = euler_to_quat(roll, pitch, yaw)

    gx = math.degrees(15 * 0.7 * math.cos(t * 0.7))
    gy = math.degrees(10 * 0.5 * math.cos(t * 0.5 + 1.0))
    gz = math.degrees(45 * 0.3 * math.cos(t * 0.3))

    ax = -math.sin(pitch)
    ay =  math.sin(roll) * math.cos(pitch)
    az =  math.cos(roll) * math.cos(pitch) + 0.05 * math.sin(t * 3)

    payload = struct.pack("<ffffffffff", qw, qx, qy, qz, gx, gy, gz, ax, ay, az)
    sock.send(make_packet(0x01, payload, next_seq(0x01), ts))

    # ── 0x03 MTF-01 (50 Hz — send every other call) ─────────────────
    if int(t * 100) % 2 == 0:
        dist  = 1.5 + 0.3 * math.sin(t * 0.4)
        flow_x = 12.0 * math.sin(t * 1.1)
        flow_y = 8.0  * math.cos(t * 0.9)
        quality = int(200 + 55 * math.sin(t * 0.2))
        payload = struct.pack("<fffB", dist, flow_x, flow_y, quality)
        sock.send(make_packet(0x03, payload, next_seq(0x03), ts))

    # ── 0x04 Radio (50 Hz) ───────────────────────────────────────────
    if int(t * 100) % 2 == 0:
        throttle = int(1200 + 300 * (0.5 + 0.5 * math.sin(t * 0.15)))
        roll_ch  = int(1500 + 400 * math.sin(t * 0.7))
        pitch_ch = int(1500 + 300 * math.sin(t * 0.5))
        yaw_ch   = int(1500 + 200 * math.sin(t * 0.3))
        channels = [roll_ch, pitch_ch, throttle, yaw_ch,
                    1500, 1500, 1500, 2000]
        channels = [max(1000, min(2000, c)) for c in channels]
        rssi = int(210 + 40 * math.sin(t * 0.1))
        payload = struct.pack("<8HB", *channels, rssi)
        sock.send(make_packet(0x04, payload, next_seq(0x04), ts))

    # ── 0x02 GPS (10 Hz) ─────────────────────────────────────────────
    if int(t * 100) % 10 == 0:
        lat  = 48.8566 + 0.0001 * math.sin(t * 0.05)
        lon  = 2.3522  + 0.0001 * math.cos(t * 0.05)
        alt  = 50.0    + 5.0 * math.sin(t * 0.1)
        spd  = 2.0     + 1.5 * abs(math.sin(t * 0.2))
        hdg  = (math.degrees(yaw) + 360) % 360
        sats = 14
        fix  = 2  # 3D fix
        payload = struct.pack("<ddfffBB", lat, lon, alt, spd, hdg, sats, fix)
        sock.send(make_packet(0x02, payload, next_seq(0x02), ts))

    # ── 0x05 Status (10 Hz) ──────────────────────────────────────────
    if int(t * 100) % 10 == 0:
        voltage = 12.4 - 0.002 * t
        current = 8.5  + 2.0 * abs(math.sin(t * 0.3))
        percent = max(0, int(100 - t * 0.05))
        # Cycle through FSM states over time
        fsm_states = ["IDLE", "ARMED", "FLYING", "STABILIZE", "ALTHOLD", "LANDING"]
        state_str  = fsm_states[int(t / 8) % len(fsm_states)]
        state_raw  = state_str.encode("utf-8")[:31].ljust(32, b"\x00")

        throttle_base = 40 + 10 * (0.5 + 0.5 * math.sin(t * 0.15))
        motors = [
            int(throttle_base + 5 * math.sin(t * 0.7 + i)) for i in range(8)
        ]
        motors = [max(0, min(100, m)) for m in motors]
        wifi_rssi = int(75 + 20 * math.sin(t * 0.07))
        payload = struct.pack("<ffB32s8BB",
            voltage, current, percent, state_raw, *motors, wifi_rssi)
        sock.send(make_packet(0x05, payload, next_seq(0x05), ts))

    # ── 0x06 PID values (1 Hz) ───────────────────────────────────────
    if int(t * 100) % 100 == 0:
        # 9 axes × (kp, ki, kd)
        axes = [
            (2.50, 0.05, 0.10),  # rate roll
            (2.50, 0.05, 0.10),  # rate pitch
            (3.00, 0.02, 0.08),  # rate yaw
            (6.00, 0.10, 0.20),  # att roll
            (6.00, 0.10, 0.20),  # att pitch
            (4.00, 0.05, 0.10),  # att yaw
            (1.20, 0.00, 0.15),  # pos x
            (1.20, 0.00, 0.15),  # pos y
            (2.00, 0.10, 0.30),  # pos z
        ]
        payload = b"".join(struct.pack("<fff", *a) for a in axes)
        sock.send(make_packet(0x06, payload, next_seq(0x06), ts))

    # ── 0x07 Log (random) ────────────────────────────────────────────
    if random.random() < 0.005:  # ~0.5% chance each tick → ~0.5 msg/s
        level, text = random.choice(LOG_MESSAGES)
        raw = text.encode("utf-8")[:127].ljust(128, b"\x00")
        payload = struct.pack("<B128s", level, raw)
        sock.send(make_packet(0x07, payload, next_seq(0x07), ts))

    # ── 0x08 Barometer (10 Hz) ───────────────────────────────────────
    if int(t * 100) % 10 == 0:
        # Pressure varies slightly around sea level, simulating altitude changes
        pressure = 101325.0 + 500.0 * math.sin(t * 0.03)
        temperature = 22.0 + 3.0 * math.sin(t * 0.02)
        # Barometric altitude from ISA approximation: matches GPS altitude simulation
        altitude = 50.0 + 5.0 * math.sin(t * 0.1)
        payload = struct.pack("<fff", pressure, temperature, altitude)
        sock.send(make_packet(0x08, payload, next_seq(0x08), ts))


# ── Main loop ─────────────────────────────────────────────────────────────────
def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.connect((GCS_HOST, GCS_PORT))
    print(f"Simulator -> {GCS_HOST}:{GCS_PORT}  (Ctrl+C to stop)")

    t0     = time.perf_counter()
    tick   = 0
    period = 1.0 / 100  # 100 Hz base rate

    while True:
        t_now = time.perf_counter() - t0
        simulate(sock, t_now)

        tick  += 1
        t_next = t0 + tick * period
        sleep  = t_next - time.perf_counter()
        if sleep > 0:
            time.sleep(sleep)

if __name__ == "__main__":
    main()
