#pragma once
#include <QByteArray>
#include <cstring>
#include "backend/Protocol.h"
#include "backend/TelemetryState.h"

// Register custom types so QSignalSpy can store them in QVariant.
Q_DECLARE_METATYPE(AttitudeData)
Q_DECLARE_METATYPE(GpsData)
Q_DECLARE_METATYPE(Mtf01Data)
Q_DECLARE_METATYPE(RadioData)
Q_DECLARE_METATYPE(StatusData)
Q_DECLARE_METATYPE(PidData)
Q_DECLARE_METATYPE(BaroData)

namespace TestHelpers {

// CRC-16/CCITT — must match PacketParser::crc16 and CommandSender::crc16.
inline uint16_t crc16(const uint8_t* data, int len) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

// Build a complete, valid datagram from a typed packet struct.
// Fills all header fields and appends a correct CRC-16.
// Call with a zero-initialised struct and set only the payload fields you care about.
template<typename T>
QByteArray buildPacket(uint8_t type, const T& payload, uint16_t seq = 1) {
    T pkt = payload;
    pkt.header.magic        = PACKET_MAGIC;
    pkt.header.version      = PACKET_VERSION;
    pkt.header.type         = type;
    pkt.header.seq          = seq;
    pkt.header.timestamp_us = 0;
    pkt.header.payload_len  = static_cast<uint16_t>(
        sizeof(T) - sizeof(PacketHeader) - sizeof(uint16_t));

    int crcLen = sizeof(PacketHeader) + pkt.header.payload_len;
    pkt.crc = crc16(reinterpret_cast<const uint8_t*>(&pkt), crcLen);

    QByteArray out(static_cast<int>(sizeof(T)), Qt::Uninitialized);
    std::memcpy(out.data(), &pkt, sizeof(T));
    return out;
}

// Build a minimal datagram where payload_len is intentionally smaller than the
// real struct payload. CRC is still valid (covers only the truncated payload),
// so the parser passes the CRC check but then rejects the packet at the size guard.
inline QByteArray buildShortPacket(uint8_t type, uint16_t fake_payload_len) {
    PacketHeader hdr{};
    hdr.magic       = PACKET_MAGIC;
    hdr.version     = PACKET_VERSION;
    hdr.type        = type;
    hdr.seq         = 1;
    hdr.payload_len = fake_payload_len;

    int totalBytes = static_cast<int>(sizeof(PacketHeader)) + fake_payload_len + 2;
    QByteArray out(totalBytes, '\0');
    std::memcpy(out.data(), &hdr, sizeof(PacketHeader));

    uint16_t crc = crc16(reinterpret_cast<const uint8_t*>(out.constData()),
                          sizeof(PacketHeader) + fake_payload_len);
    std::memcpy(out.data() + sizeof(PacketHeader) + fake_payload_len, &crc, 2);
    return out;
}

} // namespace TestHelpers
