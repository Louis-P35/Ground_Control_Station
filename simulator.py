"""
GCS Telemetry Simulator
Sends realistic fake drone telemetry to localhost:5005 via UDP.

Packet rates:
  0x01 Attitude       — 100 Hz
  0x02 GPS            —  10 Hz
  0x03 MTF-01         —  50 Hz
  0x04 Radio          —  50 Hz
  0x05 Status         —  10 Hz
  0x06 PID            —   1 Hz
  0x07 Log            — random events
  0x08 Baro           —  10 Hz
  0x09 CalibStatus    —   1 Hz (reacts to PKT_CALIB_CMD from GCS)
  0x0A FFT            —   2 Hz (6 sensor×axis combinations)
  0x0B Position (NED) —  20 Hz
"""

import socket
import select
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

# ── CalibStatus values (mirror of Protocol.h enums) ─────────────────────────
CALIB_IDLE    = 0
CALIB_RUNNING = 1
CALIB_SUCCESS = 2

# CalibAction values
CALIB_START = 0
CALIB_STOP  = 1
CALIB_SAVE  = 2

# ── Per-target calibration state machine ──────────────────────────────────────
class TargetState:
    def __init__(self, run_duration_s, has_progress, done_msg):
        self.status       = CALIB_IDLE
        self.progress     = 0
        self.message      = ""
        self.start_t      = 0.0   # wall time when RUNNING started
        self.done_t       = 0.0   # wall time when SUCCESS was set
        self.run_duration = run_duration_s
        self.has_progress = has_progress
        self.done_msg     = done_msg

calib = [
    TargetState(10.0, True,  "Accelerometer ready!"),   # 0: Accel
    TargetState(25.0, True,  "Magnetometer ready!"),    # 1: Mag
    TargetState(0.0,  False, "Level reference saved!"), # 2: Level
]

ACCEL_MSGS = ["Collecting samples...", "Keep the drone still...",
              "Measuring gravity vector..."]
MAG_MSGS   = ["Rotate the drone...", "Cover all orientations...",
              "Keep rotating slowly...", "Almost done..."]

def update_calib_states(now):
    """Advance in-progress calibrations and auto-clear SUCCESS after 5 s."""
    for i, c in enumerate(calib):
        if c.status == CALIB_RUNNING:
            elapsed = now - c.start_t
            if c.has_progress:
                c.progress = min(99, int(elapsed / c.run_duration * 100))
                if i == 0:
                    c.message = ACCEL_MSGS[int(elapsed / 3.4) % len(ACCEL_MSGS)]
                elif i == 1:
                    c.message = MAG_MSGS[int(elapsed / 6.3) % len(MAG_MSGS)]
        elif c.status == CALIB_SUCCESS:
            # Auto-reset to IDLE 5 seconds after success
            if now - c.done_t > 5.0:
                c.status   = CALIB_IDLE
                c.progress = 0
                c.message  = ""

def on_calib_cmd(target, action, now):
    """Handle an incoming PKT_CALIB_CMD from the GCS."""
    if target >= 3:
        return
    c = calib[target]
    if action == CALIB_START:
        c.status   = CALIB_RUNNING
        c.progress = 0
        c.start_t  = now
        c.message  = "Starting calibration..."
    elif action in (CALIB_STOP, CALIB_SAVE):
        c.status   = CALIB_SUCCESS
        c.progress = 100 if c.has_progress else 0
        c.done_t   = now
        c.message  = c.done_msg

def try_receive_cmd(sock, now):
    """Non-blocking read of incoming PKT_CALIB_CMD packets from the GCS."""
    readable, _, _ = select.select([sock], [], [], 0)
    if not readable:
        return
    try:
        data = sock.recv(4096)
    except OSError:
        return
    # Minimal parse: magic (2 B) + version (1 B) + type (1 B) + ... + payload
    if len(data) < 14:   # header (12) + target (1) + action (1)
        return
    magic    = struct.unpack_from("<H", data, 0)[0]
    pkt_type = data[3]
    if magic != MAGIC or pkt_type != 0x20:   # 0x20 = PKT_CALIB_CMD
        return
    target = data[12]
    action = data[13]
    on_calib_cmd(target, action, now)
    print(f"  [CMD] CalibCmd target={target} action={action}")

# ── Simulation state ──────────────────────────────────────────────────────────
# ── FFT simulation constants ─────────────────────────────────────────────────
# Mirrors Protocol.h: FFT_BIN_COUNT=128, sample_rate=1000 Hz, fft_size=256
FFT_BIN_COUNT   = 128
FFT_SAMPLE_RATE = 1000.0            # Hz
FFT_SIZE        = 256
FFT_FREQ_RES    = FFT_SAMPLE_RATE / FFT_SIZE   # ≈ 3.906 Hz per bin

# Sensor and axis identifiers — must match Protocol.h
FFT_SENSOR_ACCEL, FFT_SENSOR_GYRO = 0, 1
FFT_AXIS_X, FFT_AXIS_Y, FFT_AXIS_Z = 0, 1, 2


def _fft_spectrum(t: float, sensor: int, axis: int):
    """
    Return (raw, notch, full) lists of FFT_BIN_COUNT magnitude values.

    Simulates motor vibration with a time-varying fundamental frequency plus
    harmonics. The 'notch' spectrum removes the fundamental; 'full' additionally
    applies a soft low-pass roll-off above 80 Hz.
    """
    # Motor electrical frequency varies slightly around 120 Hz
    motor_hz = 120.0 + 8.0 * math.sin(t * 0.25)

    # Amplitude varies per sensor and axis (gyro is stronger, X axis is loudest)
    amp_sensor = 1.0 if sensor == FFT_SENSOR_GYRO else 0.7
    amp_axis   = [1.0, 0.85, 0.6][axis]
    amp        = amp_sensor * amp_axis

    raw_bins   = []
    notch_bins = []
    full_bins  = []

    for i in range(FFT_BIN_COUNT):
        hz = i * FFT_FREQ_RES

        # Noise floor — white noise with slight pink tilt
        noise = random.gauss(0.003, 0.001) + 0.004 / (1.0 + hz / 40.0)
        noise = max(0.0005, noise)

        # Gaussian peaks at motor harmonics
        def peak(f0, half_bw=3.5, peak_amp=1.0):
            return peak_amp * math.exp(-0.5 * ((hz - f0) / half_bw) ** 2)

        # --- Raw spectrum: noise + motor fundamental + harmonics ---
        raw = (noise
               + peak(motor_hz,       peak_amp=1.00 * amp)
               + peak(2 * motor_hz,   peak_amp=0.28 * amp)
               + peak(3 * motor_hz,   peak_amp=0.12 * amp)
               + peak(4 * motor_hz,   peak_amp=0.05 * amp))

        # --- Notch spectrum: fundamental attenuated ~30 dB ---
        notch = (noise
                 + peak(motor_hz,     peak_amp=0.032 * amp)  # −30 dB
                 + peak(2 * motor_hz, peak_amp=0.28  * amp)
                 + peak(3 * motor_hz, peak_amp=0.12  * amp)
                 + peak(4 * motor_hz, peak_amp=0.05  * amp))

        # --- Full (notch + low-pass at 80 Hz) ---
        lp_gain = math.exp(-max(0.0, hz - 80.0) / 25.0)
        full    = notch * lp_gain

        raw_bins.append(raw)
        notch_bins.append(notch)
        full_bins.append(full)

    return raw_bins, notch_bins, full_bins


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

    # ── 0x09 CalibStatus (1 Hz) ──────────────────────────────────────
    # Streams the current state of each calibration target. States are
    # driven by PKT_CALIB_CMD commands received from the GCS — the drone
    # never starts a calibration on its own.
    if int(t * 100) % 100 == 0:
        for i, c in enumerate(calib):
            msg_raw = c.message.encode("utf-8")[:63].ljust(64, b"\x00")
            payload = struct.pack("<BBB64s", i, c.status, c.progress, msg_raw)
            sock.send(make_packet(0x09, payload, next_seq(0x09), ts))

    # ── 0x0A FFT (2 Hz — all 6 sensor×axis combinations) ────────────
    # Each packet: sensor(B) axis(B) bin_count(H) freq_res(f) raw(128f) notch(128f) full(128f)
    if int(t * 100) % 50 == 0:
        for sensor in (FFT_SENSOR_ACCEL, FFT_SENSOR_GYRO):
            for axis in (FFT_AXIS_X, FFT_AXIS_Y, FFT_AXIS_Z):
                raw, notch, full = _fft_spectrum(t, sensor, axis)
                payload = struct.pack("<BBHf",
                                     sensor, axis,
                                     FFT_BIN_COUNT,
                                     FFT_FREQ_RES)
                payload += struct.pack(f"<{FFT_BIN_COUNT}f", *raw)
                payload += struct.pack(f"<{FFT_BIN_COUNT}f", *notch)
                payload += struct.pack(f"<{FFT_BIN_COUNT}f", *full)
                sock.send(make_packet(0x0A, payload, next_seq(0x0A), ts))

    # ── 0x0B Position (NED, 20 Hz) ───────────────────────────────────
    # A gentle circular flight path within the 20×20 m scene, with a slow
    # vertical bob. NED: north/east are horizontal, down is positive toward
    # the ground (so a drone above home has down < 0).
    if int(t * 100) % 5 == 0:
        radius = 6.0
        omega  = 0.25
        north  = radius * math.sin(t * omega)
        east   = radius * math.cos(t * omega)
        down   = -2.5 - 0.5 * math.sin(t * 0.5)          # negative = above home
        vn     =  radius * omega * math.cos(t * omega)
        ve     = -radius * omega * math.sin(t * omega)
        vd     = -0.5 * 0.5 * math.cos(t * 0.5)
        payload = struct.pack("<ffffff", north, east, down, vn, ve, vd)
        sock.send(make_packet(0x0B, payload, next_seq(0x0B), ts))


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
        update_calib_states(t_now)
        try_receive_cmd(sock, t_now)
        simulate(sock, t_now)

        tick  += 1
        t_next = t0 + tick * period
        sleep  = t_next - time.perf_counter()
        if sleep > 0:
            time.sleep(sleep)

if __name__ == "__main__":
    main()
