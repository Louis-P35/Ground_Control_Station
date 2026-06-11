#include "MagReader.h"
#include <Arduino.h>

// ---------------------------------------------------------------------------
// Register maps
// ---------------------------------------------------------------------------

namespace
{
    // HMC5883L (Honeywell) — address 0x1E
    constexpr uint8_t HMC_ADDR      = 0x1E;
    constexpr uint8_t HMC_CONFIG_A  = 0x00; // averaging / output rate
    constexpr uint8_t HMC_CONFIG_B  = 0x01; // gain
    constexpr uint8_t HMC_MODE      = 0x02; // operating mode
    constexpr uint8_t HMC_DATA      = 0x03; // X_MSB,X_LSB,Z_MSB,Z_LSB,Y_MSB,Y_LSB

    // QMC5883L (QST) — address 0x0D
    constexpr uint8_t QMC_ADDR      = 0x0D;
    constexpr uint8_t QMC_DATA      = 0x00; // X_LSB,X_MSB,Y_LSB,Y_MSB,Z_LSB,Z_MSB
    constexpr uint8_t QMC_STATUS    = 0x06; // bit0 = DRDY
    constexpr uint8_t QMC_CTRL1     = 0x09;
    constexpr uint8_t QMC_CTRL2     = 0x0A;
    constexpr uint8_t QMC_SETRESET  = 0x0B;
}

// ---------------------------------------------------------------------------
// begin — probe QMC first (most common on BN-880), then HMC
// ---------------------------------------------------------------------------

bool MagReader::begin(TwoWire& wire)
{
    m_wire = &wire;

    if (probe(QMC_ADDR) && initQmc())
    {
        m_chip = ChipType::QMC5883L;
        return true;
    }
    if (probe(HMC_ADDR) && initHmc())
    {
        m_chip = ChipType::HMC5883L;
        return true;
    }

    m_chip = ChipType::None;
    return false;
}

bool MagReader::update()
{
    switch (m_chip)
    {
        case ChipType::QMC5883L: return readQmc();
        case ChipType::HMC5883L: return readHmc();
        default:                 return false;
    }
}

const char* MagReader::chipName() const
{
    switch (m_chip)
    {
        case ChipType::QMC5883L: return "QMC5883L";
        case ChipType::HMC5883L: return "HMC5883L";
        default:                 return "none";
    }
}

// ---------------------------------------------------------------------------
// Low-level I2C helpers
// ---------------------------------------------------------------------------

bool MagReader::probe(uint8_t addr)
{
    m_wire->beginTransmission(addr);
    return m_wire->endTransmission() == 0; // 0 = device ACKed
}

bool MagReader::writeReg(uint8_t addr, uint8_t reg, uint8_t value)
{
    m_wire->beginTransmission(addr);
    m_wire->write(reg);
    m_wire->write(value);
    return m_wire->endTransmission() == 0;
}

bool MagReader::readRegs(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t count)
{
    m_wire->beginTransmission(addr);
    m_wire->write(reg);
    // Repeated start (endTransmission(false)) keeps the bus for the read.
    if (m_wire->endTransmission(false) != 0)
    {
        return false;
    }
    const uint8_t got = m_wire->requestFrom(static_cast<int>(addr), static_cast<int>(count));
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

// ---------------------------------------------------------------------------
// HMC5883L
// ---------------------------------------------------------------------------

bool MagReader::initHmc()
{
    // Config A: 8-sample average, 15 Hz output, normal measurement (0x70)
    // Config B: gain ±1.3 Ga, 1090 LSB/Gauss (0x20)
    // Mode    : continuous measurement (0x00)
    if (!writeReg(HMC_ADDR, HMC_CONFIG_A, 0x70)) return false;
    if (!writeReg(HMC_ADDR, HMC_CONFIG_B, 0x20)) return false;
    if (!writeReg(HMC_ADDR, HMC_MODE,     0x00)) return false;
    delay(10);
    return true;
}

bool MagReader::readHmc()
{
    uint8_t b[6];
    if (!readRegs(HMC_ADDR, HMC_DATA, b, 6))
    {
        return false;
    }
    // HMC data order is X, Z, Y — each big-endian (MSB first).
    m_x = static_cast<int16_t>((b[0] << 8) | b[1]);
    m_z = static_cast<int16_t>((b[2] << 8) | b[3]);
    m_y = static_cast<int16_t>((b[4] << 8) | b[5]);
    return true;
}

// ---------------------------------------------------------------------------
// QMC5883L
// ---------------------------------------------------------------------------

bool MagReader::initQmc()
{
    // Soft reset, then the mandatory SET/RESET period, then control register 1.
    writeReg(QMC_ADDR, QMC_CTRL2, 0x80); // soft reset
    delay(10);
    if (!writeReg(QMC_ADDR, QMC_SETRESET, 0x01)) return false;
    // CTRL1 = OSR 512 (00) | range 8 G (01) | ODR 200 Hz (11) | continuous (01)
    if (!writeReg(QMC_ADDR, QMC_CTRL1, 0x1D)) return false;
    delay(10);
    return true;
}

bool MagReader::readQmc()
{
    uint8_t status;
    if (!readRegs(QMC_ADDR, QMC_STATUS, &status, 1))
    {
        return false;
    }
    if ((status & 0x01) == 0) // DRDY not set — no new sample yet
    {
        return false;
    }

    uint8_t b[6];
    if (!readRegs(QMC_ADDR, QMC_DATA, b, 6))
    {
        return false;
    }
    // QMC data order is X, Y, Z — each little-endian (LSB first).
    m_x = static_cast<int16_t>(b[0] | (b[1] << 8));
    m_y = static_cast<int16_t>(b[2] | (b[3] << 8));
    m_z = static_cast<int16_t>(b[4] | (b[5] << 8));
    return true;
}
