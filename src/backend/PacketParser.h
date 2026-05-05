#pragma once
#include <QByteArray>
#include <QObject>
#include <array>
#include "Protocol.h"
#include "TelemetryState.h"

// ---------------------------------------------------------------------------
// PacketParser — parses raw UDP datagrams into typed structs.
//
// A single datagram may contain multiple back-to-back packets.
// For each recognised packet type the corresponding signal is emitted.
// CRC-16/CCITT is verified before processing; bad packets are discarded.
// ---------------------------------------------------------------------------

class PacketParser : public QObject {
    Q_OBJECT
public:
    explicit PacketParser(QObject* parent = nullptr);

    // Feed a full UDP datagram for parsing (may contain multiple packets).
    void parse(const QByteArray& datagram);

    // Per-type packet loss percentage (0–100), updated each call to parse().
    float packetLoss(uint8_t type) const;

signals:
    void attitudeReceived(AttitudeData);
    void gpsReceived     (GpsData);
    void mtf01Received   (Mtf01Data);
    void radioReceived   (RadioData);
    void statusReceived  (StatusData);
    void pidReceived     (PidData);
    void logReceived     (uint8_t level, QString text);
    void ackReceived     (uint8_t ackType, uint16_t ackSeq, uint8_t success);

private:
    // Returns true and advances offset past the packet if valid, false otherwise.
    bool tryParseOne(const uint8_t* data, int available, int& offset);

    static uint16_t crc16(const uint8_t* data, int len);

    // Sequence tracking for packet loss computation (indexed by type 0x01–0x07)
    struct SeqTracker {
        uint16_t lastSeq  = 0;
        uint32_t received = 0;
        uint32_t expected = 0;
        bool     first    = true;
    };
    std::array<SeqTracker, 8> m_seqTracker; // index = type (0 unused)
    void trackSeq(uint8_t type, uint16_t seq);
};
