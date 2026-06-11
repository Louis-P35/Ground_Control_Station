#pragma once
#include <Wire.h>
#include <cstdint>

// ---------------------------------------------------------------------------
// MagReader — I2C driver for the BN-880's on-board 3-axis magnetometer.
//
// BN-880 modules ship with one of two pin-compatible but register-incompatible
// compasses:
//   • QMC5883L  — I2C address 0x0D   (by far the most common on BN-880 clones)
//   • HMC5883L  — I2C address 0x1E   (the genuine Honeywell part)
//
// begin() probes both addresses and configures whichever chip answers, so the
// caller does not have to know which one is fitted. Wire.begin(SDA, SCL) must
// be called beforehand (SDA = GPIO21, SCL = GPIO22 on this board).
//
// Usage:
//   Wire.begin(21, 22);
//   g_mag.begin();
//   // in loop():
//   if (g_mag.update()) { ... g_mag.x(), g_mag.y(), g_mag.z() ... }
//
// x()/y()/z() return the latest raw signed counts (no calibration applied yet).
// ---------------------------------------------------------------------------

class MagReader
{
public:
    enum class ChipType : uint8_t
    {
        None,
        HMC5883L,
        QMC5883L,
    };

    // Probe the bus and initialise the detected compass. Returns true on success.
    bool begin(TwoWire& wire = Wire);

    // Read one sample into x/y/z. Returns true only when fresh data was read.
    bool update();

    ChipType    chip() const { return m_chip; }
    const char* chipName() const;

    int16_t x() const { return m_x; }
    int16_t y() const { return m_y; }
    int16_t z() const { return m_z; }

private:
    bool probe(uint8_t addr);
    bool writeReg(uint8_t addr, uint8_t reg, uint8_t value);
    bool readRegs(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t count);

    bool initHmc();
    bool initQmc();
    bool readHmc();
    bool readQmc();

    TwoWire* m_wire = nullptr;
    ChipType m_chip = ChipType::None;
    int16_t  m_x = 0;
    int16_t  m_y = 0;
    int16_t  m_z = 0;
};
