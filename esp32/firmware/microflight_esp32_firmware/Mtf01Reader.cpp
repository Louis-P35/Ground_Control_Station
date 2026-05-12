#include "Mtf01Reader.h"
#include <cstring>

// CRC extra bytes — message-type specific constant defined by the MAVLink spec
static constexpr uint8_t EXTRA_OPTICAL_FLOW    = 175; // MSG ID 100
static constexpr uint8_t EXTRA_DISTANCE_SENSOR =  85; // MSG ID 132

// ---------------------------------------------------------------------------
// begin
// ---------------------------------------------------------------------------

void Mtf01Reader::begin(HardwareSerial& serial, uint32_t baud, int rxPin, int txPin)
{
    m_serial = &serial;
    // SERIAL_8N1 = 8 data bits, no parity, 1 stop bit — matches MTF-01 spec
    m_serial->begin(baud, SERIAL_8N1, rxPin, txPin);
}

// ---------------------------------------------------------------------------
// update — drain the UART RX buffer and advance the state machine
// ---------------------------------------------------------------------------

void Mtf01Reader::update()
{
    m_newData = false;

    if (!m_serial)
    {
        return;
    }

    while (m_serial->available())
    {
        processByte(static_cast<uint8_t>(m_serial->read()));
    }
}

// ---------------------------------------------------------------------------
// processByte — advance the MAVLink v1 parser by one byte
// ---------------------------------------------------------------------------

void Mtf01Reader::processByte(uint8_t b)
{
    switch (m_state)
    {
        case State::WaitStx:
            if (b == 0xFE) // MAVLink v1 start-of-frame marker
            {
                m_state = State::Len;
            }
            break;

        case State::Len:
            m_len        = b;
            m_crc        = 0xFFFF; // reset CRC for each new frame
            m_payloadIdx = 0;
            crcUpdate(b);
            m_state = State::Seq;
            break;

        case State::Seq:
            crcUpdate(b);
            m_state = State::SysId;
            break;

        case State::SysId:
            crcUpdate(b);
            m_state = State::CompId;
            break;

        case State::CompId:
            crcUpdate(b);
            m_state = State::MsgId;
            break;

        case State::MsgId:
            m_msgId = b;
            crcUpdate(b);
            if (m_msgId != 100 && m_msgId != 132)
            {
                reset(); // unsupported message type — wait for next STX
            }
            else
            {
                // If payload is empty (should not happen for MSG 100/132), skip straight to CRC
                m_state = (m_len > 0) ? State::Payload : State::CrcL;
            }
            break;

        case State::Payload:
            // Guard against buffer overflow: only write within m_payload bounds.
            // CRC is still updated for every byte so the CRC check remains valid.
            if (m_payloadIdx < sizeof(m_payload))
            {
                m_payload[m_payloadIdx] = b;
            }
            crcUpdate(b);
            m_payloadIdx++;
            if (m_payloadIdx == m_len)
            {
                m_state = State::CrcL;
            }
            break;

        case State::CrcL:
            m_crcL  = b;
            m_state = State::CrcH;
            break;

        case State::CrcH:
        {
            // Feed the message-type extra byte into the accumulated CRC
            uint8_t extra = (m_msgId == 100) ? EXTRA_OPTICAL_FLOW : EXTRA_DISTANCE_SENSOR;
            crcUpdate(extra);

            // Received CRC is little-endian: CRC_L | (CRC_H << 8)
            uint16_t received = static_cast<uint16_t>(m_crcL)
                              | (static_cast<uint16_t>(b) << 8);
            if (m_crc == received)
            {
                dispatch();
            }
            reset();
            break;
        }
    }
}

// ---------------------------------------------------------------------------
// dispatch — extract fields from m_payload and update the output snapshot
// ---------------------------------------------------------------------------

void Mtf01Reader::dispatch()
{
    if (m_msgId == 100)
    {
        // OPTICAL_FLOW (MSG ID 100) — payload 26 bytes
        // flow_x: int16 at payload[20–21], flow_y: int16 at payload[22–23]
        // quality: uint8 at payload[25]
        if (m_len >= 26)
        {
            memcpy(&m_flowX, &m_payload[20], sizeof(int16_t));
            memcpy(&m_flowY, &m_payload[22], sizeof(int16_t));
            m_quality = m_payload[25];
        }
    }
    else // m_msgId == 132
    {
        // DISTANCE_SENSOR (MSG ID 132) — payload 14 bytes
        // current_distance: uint16 at payload[8–9], in centimetres
        if (m_len >= 10)
        {
            uint16_t raw;
            memcpy(&raw, &m_payload[8], sizeof(uint16_t));
            m_distance = raw * 0.01f;
        }
    }
    m_newData = true;
}

// ---------------------------------------------------------------------------
// reset — return to WAIT_STX without touching the output data
// ---------------------------------------------------------------------------

void Mtf01Reader::reset()
{
    m_state      = State::WaitStx;
    m_payloadIdx = 0;
}

// ---------------------------------------------------------------------------
// crcUpdate — CRC-16/X25 (reflected poly 0x8408 = reversed 0x1021), init 0xFFFF
// ---------------------------------------------------------------------------

void Mtf01Reader::crcUpdate(uint8_t b)
{
    m_crc ^= static_cast<uint16_t>(b);
    for (int i = 0; i < 8; ++i)
    {
        if (m_crc & 0x0001)
        {
            m_crc = (m_crc >> 1) ^ 0x8408;
        }
        else
        {
            m_crc >>= 1;
        }
    }
}
