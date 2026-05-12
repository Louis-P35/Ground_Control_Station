#pragma once
#include <Arduino.h>
#include <cstdint>

// ---------------------------------------------------------------------------
// Mtf01Reader — MAVLink v1 parser for the MTF-01 optical flow + rangefinder.
//
// Pins (§3.2 of ESP32_REQUIREMENTS.md):
//   RX → GPIO 25
//   TX → GPIO 26
//
// Parses two message types at 115200 baud:
//   MSG ID 100 (OPTICAL_FLOW)    : flow_x, flow_y (dpix/s), quality (0–255)
//   MSG ID 132 (DISTANCE_SENSOR) : distance (m)
//
// Usage in loop():
//   g_mtf01.update();
//   if (g_mtf01.newData()) { sendMtf01(g_mtf01); }
// ---------------------------------------------------------------------------

class Mtf01Reader
{
public:
    // rxPin/txPin override the HardwareSerial default pins (required on ESP32)
    void begin(HardwareSerial& serial, uint32_t baud, int rxPin, int txPin);

    // Drain the UART RX buffer and run the MAVLink state machine.
    // Clears newData() at the start; sets it to true if a valid frame is parsed.
    void update();

    bool    newData()  const { return m_newData;  }
    float   distance() const { return m_distance; } // metres
    int16_t flowX()    const { return m_flowX;    } // dpix/s (raw integer)
    int16_t flowY()    const { return m_flowY;    } // dpix/s (raw integer)
    uint8_t quality()  const { return m_quality;  } // 0–255

private:
    enum class State : uint8_t
    {
        WaitStx, Len, Seq, SysId, CompId, MsgId, Payload, CrcL, CrcH
    };

    void processByte(uint8_t b);
    void crcUpdate(uint8_t b);
    void dispatch();
    void reset();

    HardwareSerial* m_serial = nullptr;

    // Parser state
    State   m_state      = State::WaitStx;
    uint16_t m_crc       = 0xFFFF;
    uint8_t m_len        = 0;     // declared payload length from frame header
    uint8_t m_msgId      = 0;
    uint8_t m_payloadIdx = 0;
    uint8_t m_crcL       = 0;    // received CRC low byte (stored until CRC_H arrives)
    uint8_t m_payload[26] = {};  // buffer sized to OPTICAL_FLOW (largest message = 26 B)

    // Latest parsed values — updated in place; never reset between frames
    bool    m_newData  = false;
    float   m_distance = 0.0f;
    int16_t m_flowX    = 0;
    int16_t m_flowY    = 0;
    uint8_t m_quality  = 0;
};
