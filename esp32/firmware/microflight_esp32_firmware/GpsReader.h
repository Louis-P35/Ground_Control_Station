#pragma once
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>

// ---------------------------------------------------------------------------
// GpsReader — non-blocking NMEA parser for the BN-880 (u-blox M8).
//
// Usage:
//   g_gps.begin(Serial, 4, 2, 9600);   // RX=4, TX=2
//   // in loop():
//   g_gps.update();
//   if (g_gps.newFix()) { ... g_gps.clearFix(); }
// ---------------------------------------------------------------------------

class GpsReader
{
public:
    void begin(HardwareSerial& serial, int rxPin, int txPin, uint32_t baud = 9600);
    void update();

    bool    newFix()     const { return m_newFix; }
    void    clearFix()         { m_newFix = false; }

    bool    isValid()    const { return m_gps.location.isValid(); }
    double  latitude()   const { return m_gps.location.lat(); }
    double  longitude()  const { return m_gps.location.lng(); }
    float   altitude()   const { return m_gps.altitude.isValid()  ? (float)m_gps.altitude.meters() : 0.0f; }
    float   speed()      const { return m_gps.speed.isValid()     ? (float)m_gps.speed.mps()        : 0.0f; }
    float   heading()    const { return m_gps.course.isValid()    ? (float)m_gps.course.deg()       : 0.0f; }
    uint8_t satellites() const { return m_gps.satellites.isValid() ? (uint8_t)m_gps.satellites.value() : 0; }

    // 0 = no fix, 1 = 2D (<4 sats), 2 = 3D (≥4 sats)
    uint8_t fixType() const;

private:
    mutable TinyGPSPlus m_gps;
    HardwareSerial* m_serial = nullptr;
    bool            m_newFix = false;
};
