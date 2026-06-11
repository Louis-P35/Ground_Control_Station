#include "Bmp180Reader.h"
#include <Arduino.h>
#include <cmath>

// ---------------------------------------------------------------------------
// Register map (datasheet section 4)
// ---------------------------------------------------------------------------

namespace
{
    constexpr uint8_t BMP_ADDR        = 0x77; // Fixed I2C address
    constexpr uint8_t BMP_REG_ID      = 0xD0; // Chip ID register, must read 0x55
    constexpr uint8_t BMP_CHIP_ID     = 0x55;
    constexpr uint8_t BMP_REG_CALIB   = 0xAA; // First of 22 calibration bytes
    constexpr uint8_t BMP_REG_CTRL    = 0xF4; // Measurement control register
    constexpr uint8_t BMP_REG_DATA    = 0xF6; // MSB of the conversion result

    constexpr uint8_t BMP_CMD_TEMP    = 0x2E; // Start a temperature conversion
    constexpr uint8_t BMP_CMD_PRESS   = 0x34; // Start a pressure conversion (| OSS<<6)

    // Sea-level reference pressure for the ISA altitude formula.
    constexpr float SEA_LEVEL_PA = 101325.0f;
}

// ---------------------------------------------------------------------------
// begin — verify the chip ID, then load the factory calibration table
// ---------------------------------------------------------------------------

bool Bmp180Reader::begin(TwoWire& wire)
{
    m_wire = &wire;

    uint8_t id = 0;
    if (!readRegs(BMP_REG_ID, &id, 1) || id != BMP_CHIP_ID)
    {
        m_present = false;
        return false;
    }

    if (!readCalibration())
    {
        m_present = false;
        return false;
    }

    m_present = true;
    return true;
}

// ---------------------------------------------------------------------------
// readCalibration — read the 11 coefficients (22 bytes, big-endian) from EEPROM
// ---------------------------------------------------------------------------

bool Bmp180Reader::readCalibration()
{
    uint8_t b[22];
    if (!readRegs(BMP_REG_CALIB, b, sizeof(b)))
    {
        return false;
    }

    // Each coefficient is a 16-bit big-endian word. AC4/AC5/AC6 are unsigned.
    m_ac1 = static_cast<int16_t>((b[0]  << 8) | b[1]);
    m_ac2 = static_cast<int16_t>((b[2]  << 8) | b[3]);
    m_ac3 = static_cast<int16_t>((b[4]  << 8) | b[5]);
    m_ac4 = static_cast<uint16_t>((b[6]  << 8) | b[7]);
    m_ac5 = static_cast<uint16_t>((b[8]  << 8) | b[9]);
    m_ac6 = static_cast<uint16_t>((b[10] << 8) | b[11]);
    m_b1  = static_cast<int16_t>((b[12] << 8) | b[13]);
    m_b2  = static_cast<int16_t>((b[14] << 8) | b[15]);
    m_mb  = static_cast<int16_t>((b[16] << 8) | b[17]);
    m_mc  = static_cast<int16_t>((b[18] << 8) | b[19]);
    m_md  = static_cast<int16_t>((b[20] << 8) | b[21]);

    // A bricked / absent sensor often reads all-0x00 or all-0xFF here; reject it.
    if (m_ac1 == 0 || m_ac1 == -1 || m_ac5 == 0 || m_ac6 == 0)
    {
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// update — full measurement cycle + fixed-point compensation (datasheet 3.5)
// ---------------------------------------------------------------------------

bool Bmp180Reader::update()
{
    if (!m_present)
    {
        return false;
    }

    int32_t ut = 0; // Uncompensated temperature
    int32_t up = 0; // Uncompensated pressure
    if (!readRawTemperature(ut) || !readRawPressure(up))
    {
        return false;
    }

    // ── True temperature ─────────────────────────────────────────────────────
    int32_t x1 = ((ut - static_cast<int32_t>(m_ac6)) * static_cast<int32_t>(m_ac5)) >> 15;
    int32_t x2 = (static_cast<int32_t>(m_mc) << 11) / (x1 + m_md);
    int32_t b5 = x1 + x2;
    int32_t t  = (b5 + 8) >> 4; // Temperature in 0.1 °C

    // ── True pressure ────────────────────────────────────────────────────────
    int32_t b6 = b5 - 4000;
    x1 = (m_b2 * ((b6 * b6) >> 12)) >> 11;
    x2 = (m_ac2 * b6) >> 11;
    int32_t x3 = x1 + x2;
    int32_t b3 = (((static_cast<int32_t>(m_ac1) * 4 + x3) << OSS) + 2) >> 2;

    x1 = (m_ac3 * b6) >> 13;
    x2 = (m_b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    uint32_t b4 = (m_ac4 * static_cast<uint32_t>(x3 + 32768)) >> 15;
    uint32_t b7 = (static_cast<uint32_t>(up) - b3) * (50000u >> OSS);

    int32_t p;
    if (b7 < 0x80000000u)
    {
        p = (b7 * 2) / b4;
    }
    else
    {
        p = (b7 / b4) * 2;
    }

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p  = p + ((x1 + x2 + 3791) >> 4); // Pressure in Pa

    m_temperatureC = t / 10.0f;
    m_pressurePa   = static_cast<float>(p);
    m_altitudeM    = 44330.0f * (1.0f - powf(m_pressurePa / SEA_LEVEL_PA, 0.190295f));
    return true;
}

// ---------------------------------------------------------------------------
// Raw conversions — each blocks for the conversion time, then reads the result
// ---------------------------------------------------------------------------

bool Bmp180Reader::readRawTemperature(int32_t& ut)
{
    if (!writeReg(BMP_REG_CTRL, BMP_CMD_TEMP))
    {
        return false;
    }
    delay(5); // Datasheet: 4.5 ms max conversion time

    uint8_t b[2];
    if (!readRegs(BMP_REG_DATA, b, 2))
    {
        return false;
    }
    ut = (b[0] << 8) | b[1];
    return true;
}

bool Bmp180Reader::readRawPressure(int32_t& up)
{
    if (!writeReg(BMP_REG_CTRL, BMP_CMD_PRESS | (OSS << 6)))
    {
        return false;
    }
    // Conversion time grows with OSS: 4.5, 7.5, 13.5, 25.5 ms for OSS 0..3.
    delay(5 + (3 << OSS));

    uint8_t b[3];
    if (!readRegs(BMP_REG_DATA, b, 3))
    {
        return false;
    }
    // 19-bit result, right-justified by (8 - OSS).
    up = (((static_cast<int32_t>(b[0]) << 16) |
           (static_cast<int32_t>(b[1]) << 8)  |
            static_cast<int32_t>(b[2])) >> (8 - OSS));
    return true;
}

// ---------------------------------------------------------------------------
// Low-level I2C helpers
// ---------------------------------------------------------------------------

bool Bmp180Reader::writeReg(uint8_t reg, uint8_t value)
{
    m_wire->beginTransmission(BMP_ADDR);
    m_wire->write(reg);
    m_wire->write(value);
    return m_wire->endTransmission() == 0;
}

bool Bmp180Reader::readRegs(uint8_t reg, uint8_t* buf, uint8_t count)
{
    m_wire->beginTransmission(BMP_ADDR);
    m_wire->write(reg);
    // Repeated start keeps the bus for the read phase.
    if (m_wire->endTransmission(false) != 0)
    {
        return false;
    }
    const uint8_t got = m_wire->requestFrom(static_cast<int>(BMP_ADDR),
                                            static_cast<int>(count));
    if (got != count)
    {
        return false;
    }
    for (uint8_t i = 0; i < count; ++i)
    {
        buf[i] = m_wire->read();
    }
    return true;
}
