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

// GCS → Drone
static constexpr uint8_t PKT_SET_PID  = 0x10;
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

enum FlightMode : uint8_t {
    MANUAL    = 0x00,
    STABILIZE = 0x01,
    ALTHOLD   = 0x02,
    POSHOLD   = 0x03,
};

struct PktStatus {
    PacketHeader header;
    float   battery_voltage;  // Volts
    float   battery_current;  // Amps
    uint8_t battery_percent;  // 0–100
    uint8_t armed;            // 0=disarmed, 1=armed
    uint8_t flight_mode;      // FlightMode enum
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
