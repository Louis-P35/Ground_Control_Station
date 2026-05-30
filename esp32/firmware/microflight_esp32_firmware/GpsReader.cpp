#include "GpsReader.h"

void GpsReader::begin(HardwareSerial& serial, int rxPin, int txPin, uint32_t baud)
{
    m_serial = &serial;
    serial.begin(baud, SERIAL_8N1, rxPin, txPin);
}

void GpsReader::update()
{
    if (!m_serial) return;
    while (m_serial->available())
        m_gps.encode((char)m_serial->read());
    if (m_gps.location.isUpdated())
        m_newFix = true;
}

uint8_t GpsReader::fixType() const
{
    if (!m_gps.location.isValid()) return 0;
    return satellites() >= 4 ? 2 : 1;
}
