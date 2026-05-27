#include "SbusReader.h"
#include <Arduino.h>

// ---------------------------------------------------------------------------
// begin — configure the UART for S.Bus and open it
// ---------------------------------------------------------------------------

void SbusReader::begin(HardwareSerial& serial, int rxPin, int txPin)
{
    m_serial = &serial;
    // 100000 baud, 8E2, hardware signal inversion (S.Bus is an inverted UART)
    serial.begin(100000, SERIAL_8E2, rxPin, txPin, true /* invert */);
    Serial.printf("[SBUS] Reader started — RX=%d @ 100000 8E2 inverted\n", rxPin);
}

// ---------------------------------------------------------------------------
// update — drain the UART FIFO and assemble complete 25-byte frames
// ---------------------------------------------------------------------------

void SbusReader::update()
{
    m_newFrame = false;

    while (m_serial && m_serial->available())
    {
        uint8_t b = static_cast<uint8_t>(m_serial->read());

        if (!m_synced)
        {
            // Wait for the start byte before buffering anything
            if (b == SBUS_START_BYTE)
            {
                m_buf[0] = b;
                m_pos    = 1;
                m_synced = true;
            }
            continue;
        }

        m_buf[m_pos++] = b;

        if (m_pos == SBUS_FRAME_LEN)
        {
            // A full frame has been accumulated — validate start and end bytes
            if (m_buf[0] == SBUS_START_BYTE && m_buf[SBUS_FRAME_LEN - 1] == SBUS_END_BYTE)
            {
                decodeFrame();
                m_newFrame = true;
            }

            // Resync regardless of validity: next frame starts with a fresh start byte
            m_synced = false;
            m_pos    = 0;
        }
    }
}

// ---------------------------------------------------------------------------
// decodeFrame — unpack 16 × 11-bit channel values from bytes 1–22
//
// The 16 channels are packed consecutively in little-endian order:
//   channel n occupies bits [11n .. 11n+10] within the 176-bit payload.
// Each channel value is 11 bits wide, ranging from 0 to 2047.
// ---------------------------------------------------------------------------

void SbusReader::decodeFrame()
{
    // d points to the first payload byte (index 1, after the start byte)
    const uint8_t* d = m_buf + 1;

    m_channels[0]  = ((uint16_t)d[0]       | (uint16_t)d[1]  << 8)                            & 0x07FF;
    m_channels[1]  = ((uint16_t)d[1]  >> 3 | (uint16_t)d[2]  << 5)                            & 0x07FF;
    m_channels[2]  = ((uint16_t)d[2]  >> 6 | (uint16_t)d[3]  << 2  | (uint16_t)d[4]  << 10)  & 0x07FF;
    m_channels[3]  = ((uint16_t)d[4]  >> 1 | (uint16_t)d[5]  << 7)                            & 0x07FF;
    m_channels[4]  = ((uint16_t)d[5]  >> 4 | (uint16_t)d[6]  << 4)                            & 0x07FF;
    m_channels[5]  = ((uint16_t)d[6]  >> 7 | (uint16_t)d[7]  << 1  | (uint16_t)d[8]  << 9)   & 0x07FF;
    m_channels[6]  = ((uint16_t)d[8]  >> 2 | (uint16_t)d[9]  << 6)                            & 0x07FF;
    m_channels[7]  = ((uint16_t)d[9]  >> 5 | (uint16_t)d[10] << 3)                            & 0x07FF;
    m_channels[8]  = ((uint16_t)d[11]      | (uint16_t)d[12] << 8)                            & 0x07FF;
    m_channels[9]  = ((uint16_t)d[12] >> 3 | (uint16_t)d[13] << 5)                            & 0x07FF;
    m_channels[10] = ((uint16_t)d[13] >> 6 | (uint16_t)d[14] << 2  | (uint16_t)d[15] << 10)  & 0x07FF;
    m_channels[11] = ((uint16_t)d[15] >> 1 | (uint16_t)d[16] << 7)                            & 0x07FF;
    m_channels[12] = ((uint16_t)d[16] >> 4 | (uint16_t)d[17] << 4)                            & 0x07FF;
    m_channels[13] = ((uint16_t)d[17] >> 7 | (uint16_t)d[18] << 1  | (uint16_t)d[19] << 9)   & 0x07FF;
    m_channels[14] = ((uint16_t)d[19] >> 2 | (uint16_t)d[20] << 6)                            & 0x07FF;
    m_channels[15] = ((uint16_t)d[20] >> 5 | (uint16_t)d[21] << 3)                            & 0x07FF;

    // Flags byte (byte 23 of the frame = d[22])
    uint8_t flags = d[22];
    m_lostFrame = (flags & 0x04) != 0; // bit 2
    m_failsafe  = (flags & 0x08) != 0; // bit 3
}
