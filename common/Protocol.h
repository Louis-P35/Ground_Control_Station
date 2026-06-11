#pragma once
#include <cstdint>

// ---------------------------------------------------------------------------
// Common packet header — prepended to every packet in both directions
// ---------------------------------------------------------------------------
#pragma pack(push, 1)

struct PacketHeader {
    uint16_t magic;        // Always 0xABCD
    uint8_t  version;      // Protocol version = 0x01
    uint8_t  type;         // Packet type
    uint32_t timestamp_us; // Microseconds since drone boot
    uint16_t seq;          // Sequence number (per packet type)
    uint16_t payload_len;  // Byte length of payload (excluding header and CRC)
};

static constexpr uint16_t PACKET_MAGIC   = 0xABCD;
static constexpr uint8_t  PACKET_VERSION = 0x01;

// ---------------------------------------------------------------------------
// Drone → GCS packet type IDs
// ---------------------------------------------------------------------------
static constexpr uint8_t PKT_ATTITUDE = 0x01;
static constexpr uint8_t PKT_GPS      = 0x02;
static constexpr uint8_t PKT_MTF01    = 0x03;
static constexpr uint8_t PKT_RADIO    = 0x04;
static constexpr uint8_t PKT_STATUS   = 0x05;
static constexpr uint8_t PKT_PID      = 0x06;
static constexpr uint8_t PKT_LOG      = 0x07;
static constexpr uint8_t PKT_BARO          = 0x08;
static constexpr uint8_t PKT_CALIB_STATUS  = 0x09; // Drone → GCS: calibration progress
static constexpr uint8_t PKT_FFT           = 0x0A; // Drone → GCS: FFT spectrum (sensor + axis)
static constexpr uint8_t PKT_POSITION      = 0x0B; // Drone → GCS: NWU position + velocity estimate
static constexpr uint8_t PKT_MAG           = 0x0C; // Drone → GCS: filtered magnetometer (compass)

// GCS → Drone
static constexpr uint8_t PKT_SET_PID       = 0x10;
static constexpr uint8_t PKT_CALIB_CMD     = 0x20; // Start / stop / save a calibration
// Drone → GCS (ACK)
static constexpr uint8_t PKT_ACK      = 0x11;

// ---------------------------------------------------------------------------
// Drone → GCS payloads
// ---------------------------------------------------------------------------

struct PktAttitude {
    PacketHeader header;
    float qw, qx, qy, qz; // Unit quaternion
    float gx, gy, gz;      // Angular velocity °/s
    float ax, ay, az;      // Acceleration m/s²
    uint16_t crc;
};

struct PktGps {
    PacketHeader header;
    double  latitude;     // Decimal degrees
    double  longitude;    // Decimal degrees
    float   altitude_m;   // Meters above sea level
    float   speed_ms;     // Ground speed m/s
    float   heading_deg;  // Course over ground, degrees
    uint8_t satellites;
    uint8_t fix_type;     // 0=none, 1=2D, 2=3D
    uint16_t crc;
};

struct PktMtf01 {
    PacketHeader header;
    float   distance_m; // Laser rangefinder, meters
    float   flow_x;     // Optical flow X, px/s
    float   flow_y;     // Optical flow Y, px/s
    uint8_t quality;    // Flow quality 0–255
    uint16_t crc;
};

struct PktRadio {
    PacketHeader header;
    uint16_t channels[8]; // Raw channel values (typically 1000–2000 µs)
    uint8_t  rssi;        // Receiver RSSI 0–255
    uint16_t crc;
};

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

struct PktLog {
    PacketHeader header;
    uint8_t level;      // 0=DEBUG, 1=INFO, 2=WARN, 3=ERROR
    char    text[128];  // Null-terminated UTF-8 string
    uint16_t crc;
};

struct PktBaro {
    PacketHeader header;
    float pressure_pa;   // Atmospheric pressure in Pascals
    float temperature_c; // Temperature in Celsius
    float altitude_m;    // Barometric altitude in meters (derived from pressure)
    uint16_t crc;
};

// 0x0B — Drone → GCS: fused position estimate in the local NWU frame.
// The origin is the home/arming point. North/West are horizontal axes,
// Up is positive away from the ground (so a climbing drone has up_m > 0).
// NWU matches the world frame in which the drone's attitude quaternion is
// expressed, so the GCS uses a single convention end to end.
struct PktPosition {
    PacketHeader header;
    float north_m;  // Position north of home, meters
    float west_m;   // Position west  of home, meters
    float up_m;     // Position above home, meters (positive = above origin)
    float vel_north; // Velocity north, m/s
    float vel_west;  // Velocity west,  m/s
    float vel_up;    // Velocity up,    m/s
    uint16_t crc;
};

// 0x0C — Drone → GCS: filtered magnetometer reading from the FC, used to drive
// the compass widget. Values are raw signed counts in the sensor frame; the GCS
// derives the heading from the horizontal components.
struct PktMag {
    PacketHeader header;
    int16_t x; // Magnetometer X, raw counts
    int16_t y; // Magnetometer Y, raw counts
    int16_t z; // Magnetometer Z, raw counts
    uint16_t crc;
};

// ---------------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------------

// Which sensor is being calibrated
enum CalibTarget : uint8_t {
    CALIB_ACCEL = 0, // Accelerometer
    CALIB_MAG   = 1, // Magnetometer
    CALIB_LEVEL = 2, // Zero attitude (stationary hover reference)
};

// Action sent from GCS to the drone
enum CalibAction : uint8_t {
    CALIB_START = 0, // Begin the calibration sequence
    CALIB_STOP  = 1, // Abort without saving
    CALIB_SAVE  = 2, // Stop acquisition and persist results
};

// Status values reported by the drone
enum CalibStatusValue : uint8_t {
    CALIB_IDLE    = 0,
    CALIB_RUNNING = 1,
    CALIB_SUCCESS = 2,
    CALIB_FAILED  = 3,
};

// 0x09 — Drone → GCS: calibration progress for one sensor
struct PktCalibStatus {
    PacketHeader header;
    uint8_t target;      // CalibTarget
    uint8_t status;      // CalibStatusValue
    uint8_t progress;    // 0-100 %
    char    message[64]; // Null-terminated human-readable status message
    uint16_t crc;
};

// ---------------------------------------------------------------------------
// FFT packet constants
// ---------------------------------------------------------------------------

static constexpr uint16_t FFT_BIN_COUNT = 128; // Max frequency bins per packet

// Sensor identifiers (PktFft::sensor)
static constexpr uint8_t FFT_SENSOR_ACCEL = 0;
static constexpr uint8_t FFT_SENSOR_GYRO  = 1;

// Axis identifiers (PktFft::axis)
static constexpr uint8_t FFT_AXIS_X = 0;
static constexpr uint8_t FFT_AXIS_Y = 1;
static constexpr uint8_t FFT_AXIS_Z = 2;

// 0x0A — Drone → GCS: FFT spectrum for one sensor/axis combination.
// The drone computes three parallel spectra so the GCS can visualise the
// effect of each filter stage without a round-trip.
struct PktFft
{
    PacketHeader header;
    uint8_t  sensor;               // FFT_SENSOR_ACCEL or FFT_SENSOR_GYRO
    uint8_t  axis;                 // FFT_AXIS_X, FFT_AXIS_Y, or FFT_AXIS_Z
    uint16_t bin_count;            // Valid bins in each array (≤ FFT_BIN_COUNT)
    float    freq_resolution_hz;   // Hz per bin (= sample_rate / fft_size)
    float    raw  [FFT_BIN_COUNT]; // Raw signal FFT magnitudes
    float    notch[FFT_BIN_COUNT]; // Notch-filtered signal FFT magnitudes
    float    full [FFT_BIN_COUNT]; // Notch+pass-filtered signal FFT magnitudes
    uint16_t crc;
};

// ---------------------------------------------------------------------------
// GCS → Drone packets
// ---------------------------------------------------------------------------

enum PidAxisId : uint8_t {
    RATE_ROLL = 0, RATE_PITCH, RATE_YAW,
    ATT_ROLL, ATT_PITCH, ATT_YAW,
    POS_X, POS_Y, POS_Z
};

struct PktSetPid {
    PacketHeader header;
    uint8_t axis_id; // PidAxisId
    float   kp, ki, kd;
    uint16_t crc;
};

// 0x20 — GCS → Drone: start/stop/save a calibration sequence
struct PktCalibCmd {
    PacketHeader header;
    uint8_t target; // CalibTarget
    uint8_t action; // CalibAction
    uint16_t crc;
};

// ---------------------------------------------------------------------------
// Drone → GCS ACK
// ---------------------------------------------------------------------------

struct PktAck {
    PacketHeader header;
    uint8_t  ack_type; // Type of the packet being acknowledged
    uint16_t ack_seq;  // Sequence number of the acknowledged packet
    uint8_t  success;  // 1=ok, 0=error
    uint16_t crc;
};

#pragma pack(pop)
