#pragma once
#include <Arduino.h>
#include <cstdint>

// ---------------------------------------------------------------------------
// SbusReader — Non-blocking S.Bus frame parser.
//
// Pin (§3.4 of ESP32_REQUIREMENTS.md):
//   RX → GPIO 16
//   TX → GPIO 17  (unused; TX pin required by ESP32 UART driver)
//
// S.Bus electrical spec:
//   - 100000 baud, 8 data bits, even parity, 2 stop bits (8E2)
//   - Signal is logically inverted — the ESP32 UART hardware inversion is used
//   - One 25-byte frame every ~14 ms at 70 Hz
//
// Frame layout:
//   byte  0    : 0x0F  (start byte)
//   bytes 1–22 : 16 channels × 11 bits, packed little-endian (LSB first)
//   byte 23    : flags (bit 0 = ch17, bit 1 = ch18, bit 2 = lost frame, bit 3 = failsafe)
//   byte 24    : 0x00  (end byte)
//
// Raw channel values: 0–2047 (11 bits). Typical servo range: 172–1811.
//
// Usage in loop():
//   g_sbus.update();
//   if (g_sbus.newFrame()) { sendRadio(g_sbus); }
// ---------------------------------------------------------------------------

static constexpr int     SBUS_FRAME_LEN    = 25;
static constexpr uint8_t SBUS_START_BYTE   = 0x0F;
static constexpr uint8_t SBUS_END_BYTE     = 0x00;
static constexpr int     SBUS_NUM_CHANNELS = 16;

// Typical PWM range that maps to 1000–2000 µs on standard servos
static constexpr int SBUS_RAW_MIN = 172;
static constexpr int SBUS_RAW_MAX = 1811;

class SbusReader
{
public:
    // Call once in setup() — opens the UART with 8E2 + hardware signal inversion
    void begin(HardwareSerial& serial, int rxPin, int txPin);

    // Drain the UART FIFO and parse complete 25-byte frames.
    // newFrame() returns true for exactly one call per decoded frame.
    void update();

    // True if a new frame was decoded during the last update() call
    bool newFrame() const { return m_newFrame; }

    // Raw 11-bit channel value [0, 2047], index 0–15
    uint16_t channel(int idx) const
    {
        return (idx >= 0 && idx < SBUS_NUM_CHANNELS) ? m_channels[idx] : 0;
    }

    // Failsafe and lost-frame flags decoded from byte 23
    bool failsafe()  const { return m_failsafe;  }
    bool lostFrame() const { return m_lostFrame; }

private:
    void decodeFrame();

    HardwareSerial* m_serial  = nullptr;

    // Byte accumulation buffer — filled one byte at a time from the UART FIFO
    uint8_t m_buf[SBUS_FRAME_LEN] = {};
    int     m_pos    = 0;      // next write index in m_buf
    bool    m_synced = false;  // true once we have seen a valid start byte

    // Latest decoded channel data
    uint16_t m_channels[SBUS_NUM_CHANNELS] = {};
    bool     m_failsafe  = false;
    bool     m_lostFrame = false;
    bool     m_newFrame  = false;
};
