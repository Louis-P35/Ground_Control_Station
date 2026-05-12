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
//     [magic 2B] [has_cmd 1B] [cmd_type 1B] [cmd payload 27B] [CRC16 2B] [pad 223B]
// ---------------------------------------------------------------------------

static constexpr size_t   SPI_FRAME_SIZE      = 256;
static constexpr uint16_t SPI_MAGIC_FC_TO_ESP = 0xBEEF;
static constexpr uint16_t SPI_MAGIC_ESP_TO_FC = 0xCAFE;

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
        SpiPayloadAttitude    attitude;            //  40 bytes
        SpiPayloadStatus      status;              //  50 bytes
        SpiPayloadPid         pid;                 // 108 bytes
        SpiPayloadCalibStatus calib;               //  67 bytes
        SpiPayloadLog         log;                 // 129 bytes
        uint8_t               raw[SPI_MAX_PAYLOAD]; // 248 bytes — sizes the union
    } payload;               // 248 bytes
    uint16_t crc;            //   2 bytes
    uint8_t  pad[2];         //   2 bytes  → total = 256
};
static_assert(sizeof(SpiRxFrame) == SPI_FRAME_SIZE, "SpiRxFrame must be 256 bytes");

// ESP32 → FC (MISO) — carries at most one pending GCS command per transaction
struct SpiTxFrame
{
    uint16_t magic;    // SPI_MAGIC_ESP_TO_FC
    uint8_t  has_cmd;  // 1 = a valid GCS command is in cmd; FC ignores cmd if 0
    uint8_t  cmd_type; // PKT_SET_PID (0x10) or PKT_CALIB_CMD (0x20)
    union
    {
        uint8_t set_pid_raw  [sizeof(PktSetPid)];   // 27 bytes
        uint8_t calib_cmd_raw[sizeof(PktCalibCmd)]; // 16 bytes
    } cmd;             //  27 bytes (sized to the largest command)
    uint16_t crc;      //   2 bytes — covers magic + has_cmd + cmd_type + cmd
    uint8_t  pad[SPI_FRAME_SIZE - 2 - 1 - 1 - sizeof(PktSetPid) - 2]; // 223 bytes
};
static_assert(sizeof(SpiTxFrame) == SPI_FRAME_SIZE, "SpiTxFrame must be 256 bytes");

#pragma pack(pop)
