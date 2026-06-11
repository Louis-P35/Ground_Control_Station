#pragma once
#include <Wire.h>
#include <cstdint>

// ---------------------------------------------------------------------------
// Bmp180Reader — I2C driver for the Bosch BMP180 barometric pressure sensor.
//
// The BMP180 lives at the fixed I2C address 0x77 and shares the bus with the
// BN-880 compass (SDA = GPIO21, SCL = GPIO22 on this board). Wire.begin(SDA,
// SCL) must be called once before begin().
//
// The chip exposes 11 factory calibration coefficients in its EEPROM that must
// be read once at start-up. Each measurement is a two-step, blocking sequence:
//   1. trigger an uncompensated temperature reading  (~4.5 ms)
//   2. trigger an uncompensated pressure    reading  (~4.5 ms at OSS = 0)
// The raw values are then run through the fixed-point compensation formulas
// from the datasheet to obtain true temperature (°C) and pressure (Pa).
//
// Usage:
//   Wire.begin(21, 22);
//   g_baro.begin();
//   // in loop(), throttled (the read blocks ~9 ms):
//   if (g_baro.update())
//   {
//       g_baro.pressurePa();      // Pa
//       g_baro.temperatureC();    // °C
//       g_baro.altitudeM();       // m above sea level (ISA, 1013.25 hPa)
//   }
// ---------------------------------------------------------------------------

class Bmp180Reader
{
public:
    // Probe the bus, verify the chip ID and load the calibration table.
    // Returns true on success.
    bool begin(TwoWire& wire = Wire);

    // Run one full temperature + pressure measurement cycle and update the
    // cached values. Returns true only when a fresh sample was read.
    // NOTE: this call blocks for roughly 9 ms (two conversions).
    bool update();

    bool  isPresent()     const { return m_present; }
    float pressurePa()    const { return m_pressurePa; }
    float temperatureC()  const { return m_temperatureC; }
    float altitudeM()     const { return m_altitudeM; }

private:
    // Oversampling setting (0–3). 0 = single sample, fastest (4.5 ms) — fine
    // for a first debug pass where we just want live numbers on the GCS.
    static constexpr uint8_t OSS = 0;

    bool readCalibration();
    bool writeReg(uint8_t reg, uint8_t value);
    bool readRegs(uint8_t reg, uint8_t* buf, uint8_t count);

    // Raw uncompensated readings from the sensor.
    bool readRawTemperature(int32_t& ut);
    bool readRawPressure(int32_t& up);

    TwoWire* m_wire = nullptr;
    bool     m_present = false;

    // Factory calibration coefficients (datasheet section 3.4).
    int16_t  m_ac1 = 0, m_ac2 = 0, m_ac3 = 0;
    uint16_t m_ac4 = 0, m_ac5 = 0, m_ac6 = 0;
    int16_t  m_b1 = 0, m_b2 = 0;
    int16_t  m_mb = 0, m_mc = 0, m_md = 0;

    float m_pressurePa   = 0.0f;
    float m_temperatureC = 0.0f;
    float m_altitudeM    = 0.0f;
};
