#pragma once
#include <cstdint>
#include "../../../common/Protocol.h" // PidAxis, PktSetPid, PktCalibCmd — shared with GCS

// ---------------------------------------------------------------------------
// SPI frame format — internal protocol between the Flight Controller (master)
// and the ESP32 (slave).
//
// Completely separate from the GCS UDP protocol (Protocol.h). Its sole
// purpose is to carry data across the SPI bus between the two MCUs.
//
// Both directions use a fixed SPI_FRAME_SIZE so the FC always clocks exactly
// that many bytes per transaction. Unused payload bytes are zero-padded.
//
// Layout (256 bytes in each direction):
//
//   FC → ESP32 (MOSI):
//     [SpiFrameHeader 4B] [payload up to 248B] [CRC16 2B] [pad 2B]
//
//   ESP32 → FC (MISO):
//     [magic 2B] [has_cmd 1B] [cmd_type 1B] [cmd 27B] [CRC16 2B]
//     [has_sbus 1B] [sbus_raw 32B]
//     [has_gps 1B] [gps 30B] [has_mtf01 1B] [mtf01 9B] [pad 149B]
//
//     The CRC covers only the command section (bytes 0–30, unchanged from the
//     original protocol so the FC does not need to update its CRC check).
//     The raw S.Bus / GPS / MTF-01 sensor sections live after the CRC in the
//     padding area: the ESP32 reads those peripherals and exposes their values
//     so the FC can consume them, exactly like the raw S.Bus channels. The FC
//     then echoes processed GPS / MTF-01 back over MOSI for the ESP32 to relay
//     to the GCS.
// ---------------------------------------------------------------------------

static constexpr size_t   SPI_FRAME_SIZE      = 256;
static constexpr uint16_t SPI_MAGIC_FC_TO_ESP = 0xBEEF;
static constexpr uint16_t SPI_MAGIC_ESP_TO_FC = 0xCAFE;

// Number of S.Bus channels carried in the MISO frame
static constexpr int SBUS_SPI_CHANNELS = 16;

// ---------------------------------------------------------------------------
// FC → ESP32 frame types (carried in SpiFrameHeader::type)
// ---------------------------------------------------------------------------
enum class SpiFrameType : uint8_t
{
    Attitude    = 0x01, // 100 Hz — quaternion, gyro, accel
    Status      = 0x02, //  10 Hz — battery, FSM state, motors
    Pid         = 0x03, //   1 Hz — all 9 PID axes
    CalibStatus = 0x04, // on event — one calibration target
    Log         = 0x05, // on event — one log message
    Radio       = 0x06, //  50 Hz — processed radio channels from FC (1000–2000 µs)
    Gps         = 0x07, //   1 Hz — GPS fix echoed back by the FC
    Mtf01       = 0x08, //  ~rate — MTF-01 flow/range echoed back by the FC
    Mag         = 0x09, //   5 Hz — magnetometer echoed back by the FC
};

// ---------------------------------------------------------------------------
// Payload structs (FC → ESP32, carried in the payload area of SpiRxFrame)
// ---------------------------------------------------------------------------
#pragma pack(push, 1)

struct SpiFrameHeader
{
    uint16_t     magic;       // SPI_MAGIC_FC_TO_ESP
    SpiFrameType type;
    uint8_t      payload_len; // valid bytes immediately after this header
};
static_assert(sizeof(SpiFrameHeader) == 4, "SpiFrameHeader must be 4 bytes");

struct SpiPayloadAttitude
{
    float qw, qx, qy, qz; // unit quaternion
    float gx, gy, gz;     // angular velocity, deg/s
    float ax, ay, az;     // acceleration, m/s²
};

struct SpiPayloadStatus
{
    float   battery_voltage;  // V
    float   battery_current;  // A
    uint8_t battery_percent;  // 0–100
    char    state[32];        // null-terminated FSM state string
    uint8_t motor_percent[8]; // 0–100 per motor
    uint8_t wifi_rssi;        // 0–100
};

struct SpiPayloadPid
{
    PidAxis rate_roll,  rate_pitch,  rate_yaw;
    PidAxis att_roll,   att_pitch,   att_yaw;
    PidAxis pos_x,      pos_y,       pos_z;
};

struct SpiPayloadCalibStatus
{
    uint8_t target;      // CalibTarget
    uint8_t status;      // CalibStatusValue
    uint8_t progress;    // 0–100 %
    char    message[64]; // null-terminated human-readable message
};

struct SpiPayloadLog
{
    uint8_t level;      // 0=DEBUG 1=INFO 2=WARN 3=ERROR
    char    text[128];  // null-terminated UTF-8 string
};

// Processed radio channels sent by the FC after mixing / calibration.
// Values are in standard PWM microseconds (typically 1000–2000).
struct SpiPayloadRadio
{
    uint16_t channels[SBUS_SPI_CHANNELS]; // Processed PWM, 1000–2000 µs
    uint8_t  rssi;                        // Signal quality 0–255
};

// GPS fix. Used in BOTH directions: the ESP32 puts the raw BN-880 reading on
// MISO for the FC, and the FC echoes it back on MOSI for the ESP32 to relay
// to the GCS (mirrors the S.Bus → Radio round trip).
struct SpiPayloadGps
{
    double  latitude;     // Decimal degrees
    double  longitude;    // Decimal degrees
    float   altitude_m;   // Metres above sea level
    float   speed_ms;     // Ground speed m/s
    float   heading_deg;  // Course over ground, degrees
    uint8_t satellites;
    uint8_t fix_type;     // 0=none, 1=2D, 2=3D
};

// MTF-01 optical-flow + rangefinder. Same round-trip role as SpiPayloadGps.
// Flow is carried as the raw signed dpix/s integers from the sensor.
struct SpiPayloadMtf01
{
    float   distance_m; // Laser rangefinder, metres
    int16_t flow_x;     // Optical flow X, raw dpix/s
    int16_t flow_y;     // Optical flow Y, raw dpix/s
    uint8_t quality;    // Flow quality 0–255
};

// BN-880 magnetometer (HMC5883L / QMC5883L). Same round-trip role as the GPS:
// the ESP32 puts the raw counts on MISO, the FC echoes them back on MOSI.
struct SpiPayloadMag
{
    int16_t x; // Raw magnetometer counts
    int16_t y;
    int16_t z;
};

// ---------------------------------------------------------------------------
// Full fixed-size frames
// ---------------------------------------------------------------------------
static constexpr size_t SPI_MAX_PAYLOAD =
    SPI_FRAME_SIZE - sizeof(SpiFrameHeader) - 2 /*crc*/ - 2 /*pad*/; // = 248

// FC → ESP32 (MOSI)
struct SpiRxFrame
{
    SpiFrameHeader header;   //   4 bytes
    union
    {
        SpiPayloadAttitude    attitude;              //  40 bytes
        SpiPayloadStatus      status;               //  50 bytes
        SpiPayloadPid         pid;                  // 108 bytes
        SpiPayloadCalibStatus calib;                //  67 bytes
        SpiPayloadLog         log;                  // 129 bytes
        SpiPayloadRadio       radio;                //  33 bytes
        SpiPayloadGps         gps;                  //  30 bytes
        SpiPayloadMtf01       mtf01;                //   9 bytes
        SpiPayloadMag         mag;                  //   6 bytes
        uint8_t               raw[SPI_MAX_PAYLOAD]; // 248 bytes — sizes the union
    } payload;               // 248 bytes
    uint16_t crc;            //   2 bytes
    uint8_t  pad[2];         //   2 bytes  → total = 256
};
static_assert(sizeof(SpiRxFrame) == SPI_FRAME_SIZE, "SpiRxFrame must be 256 bytes");

// ESP32 → FC (MISO)
//
// The frame is split into independent sections:
//
//   [0..32]   — Command section (GCS command forwarding, CRC-protected)
//   [33..65]  — S.Bus section   (raw receiver values, appended in padding)
//   [66..113] — GPS + MTF-01 + magnetometer sections (raw sensor values)
//   [114..255]— Zero padding
//
// The CRC covers only the command section so that the FC does not need to
// update its CRC check to gain access to the raw sensor data.
struct SpiTxFrame
{
    // ── Command section (existing, CRC-protected) ────────────────────────────
    uint16_t magic;    // SPI_MAGIC_ESP_TO_FC
    uint8_t  has_cmd;  // 1 = a valid GCS command is in cmd; FC ignores cmd if 0
    uint8_t  cmd_type; // PKT_SET_PID (0x10) or PKT_CALIB_CMD (0x20)
    union
    {
        uint8_t set_pid_raw  [sizeof(PktSetPid)];   // 27 bytes
        uint8_t calib_cmd_raw[sizeof(PktCalibCmd)]; // 16 bytes
    } cmd;             // 27 bytes (sized to the largest command)
    uint16_t crc;      //  2 bytes — covers magic + has_cmd + cmd_type + cmd

    // ── S.Bus section (raw receiver values for the FC) ───────────────────────
    uint8_t  has_sbus;                     // 1 = sbus_raw[] contains a valid frame
    uint16_t sbus_raw[SBUS_SPI_CHANNELS];  // Raw 11-bit SBUS values (0–2047)

    // ── GPS section (raw BN-880 fix for the FC) ──────────────────────────────
    uint8_t       has_gps;                 // 1 = gps holds a valid fix
    SpiPayloadGps gps;

    // ── MTF-01 section (raw optical-flow / range for the FC) ──────────────────
    uint8_t         has_mtf01;             // 1 = mtf01 holds a fresh reading
    SpiPayloadMtf01 mtf01;

    // ── Magnetometer section (raw compass counts for the FC) ──────────────────
    uint8_t       has_mag;                 // 1 = mag holds a fresh reading
    SpiPayloadMag mag;

    // ── Padding to reach 256 bytes ───────────────────────────────────────────
    uint8_t  pad[SPI_FRAME_SIZE
                 - 2                       // magic
                 - 1                       // has_cmd
                 - 1                       // cmd_type
                 - sizeof(PktSetPid)       // cmd (27 B)
                 - 2                       // crc
                 - 1                       // has_sbus
                 - SBUS_SPI_CHANNELS * 2   // sbus_raw (32 B)
                 - 1                       // has_gps
                 - sizeof(SpiPayloadGps)   // gps (30 B)
                 - 1                       // has_mtf01
                 - sizeof(SpiPayloadMtf01) // mtf01 (9 B)
                 - 1                       // has_mag
                 - sizeof(SpiPayloadMag)]; // mag (6 B)  → 142 bytes
};
static_assert(sizeof(SpiTxFrame) == SPI_FRAME_SIZE, "SpiTxFrame must be 256 bytes");

#pragma pack(pop)
