#include "PacketParser.h"
#include "AppLogger.h"
#include <cstring>
#include <QtMath>

PacketParser::PacketParser(QObject* parent) : QObject(parent) {}

// ---------------------------------------------------------------------------
// CRC-16/CCITT (polynomial 0x1021, init 0xFFFF)
// ---------------------------------------------------------------------------
uint16_t PacketParser::crc16(const uint8_t* data, int len) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

// ---------------------------------------------------------------------------
// parse — entry point, iterates over all packets in the datagram
// ---------------------------------------------------------------------------
void PacketParser::parse(const QByteArray& datagram) {
    const auto* data = reinterpret_cast<const uint8_t*>(datagram.constData());
    int         size = datagram.size();
    int         offset = 0;

    while (offset < size) {
        if (!tryParseOne(data, size - offset, offset))
            break; // Cannot recover — discard remainder
    }
}

// ---------------------------------------------------------------------------
// tryParseOne — attempts to parse one packet starting at data[offset]
// Returns true if a packet (valid or not) was consumed; false on fatal error.
// ---------------------------------------------------------------------------
bool PacketParser::tryParseOne(const uint8_t* data, int available, int& offset) {
    // Need at least a full header
    if (available < static_cast<int>(sizeof(PacketHeader))) return false;

    const auto* hdr = reinterpret_cast<const PacketHeader*>(data + offset);

    // Magic / version check — log at most once per second to avoid flooding
    if (hdr->magic != PACKET_MAGIC || hdr->version != PACKET_VERSION) {
        if (!m_syncWarnTimerStarted || m_syncWarnTimer.elapsed() > 1000) {
            AppLogger::warn("PacketParser: sync lost (bad magic/version), resyncing");
            m_syncWarnTimer.restart();
            m_syncWarnTimerStarted = true;
        }
        ++offset; // Advance one byte and try again
        return true;
    }

    int totalSize = sizeof(PacketHeader) + hdr->payload_len + sizeof(uint16_t);
    if (available < totalSize) return false; // Incomplete — wait for more data

    // CRC covers header + payload
    uint16_t computed = crc16(data + offset, sizeof(PacketHeader) + hdr->payload_len);
    uint16_t received;
    std::memcpy(&received, data + offset + sizeof(PacketHeader) + hdr->payload_len, sizeof(uint16_t));

    if (computed != received) {
        AppLogger::warn(QString("PacketParser: CRC mismatch (type=0x%1, size=%2, got=0x%3, expected=0x%4)")
                        .arg(hdr->type,    2, 16, QChar('0'))
                        .arg(totalSize)
                        .arg(received,  4, 16, QChar('0'))
                        .arg(computed,  4, 16, QChar('0')));
        offset += totalSize;
        return true;
    }

    // Track sequence numbers for types 0x01–0x0A
    if (hdr->type >= 0x01 && hdr->type <= 0x0A)
        trackSeq(hdr->type, hdr->seq);

    // Dispatch by type
    const uint8_t* pkt = data + offset;
    switch (hdr->type) {
        case PKT_ATTITUDE: {
            if (totalSize < static_cast<int>(sizeof(PktAttitude))) break;
            PktAttitude p; std::memcpy(&p, pkt, sizeof(p));
            AttitudeData d;
            d.qw = p.qw; d.qx = p.qx; d.qy = p.qy; d.qz = p.qz;
            d.gx = p.gx; d.gy = p.gy; d.gz = p.gz;
            d.ax = p.ax; d.ay = p.ay; d.az = p.az;
            emit attitudeReceived(d);
            break;
        }
        case PKT_GPS: {
            if (totalSize < static_cast<int>(sizeof(PktGps))) break;
            PktGps p; std::memcpy(&p, pkt, sizeof(p));
            GpsData d;
            d.latitude    = p.latitude;
            d.longitude   = p.longitude;
            d.altitude_m  = p.altitude_m;
            d.speed_ms    = p.speed_ms;
            d.heading_deg = p.heading_deg;
            d.satellites  = p.satellites;
            d.fix_type    = p.fix_type;
            emit gpsReceived(d);
            break;
        }
        case PKT_MTF01: {
            if (totalSize < static_cast<int>(sizeof(PktMtf01))) break;
            PktMtf01 p; std::memcpy(&p, pkt, sizeof(p));
            Mtf01Data d;
            d.distance_m = p.distance_m;
            d.flow_x     = p.flow_x;
            d.flow_y     = p.flow_y;
            d.quality    = p.quality;
            emit mtf01Received(d);
            break;
        }
        case PKT_RADIO: {
            if (totalSize < static_cast<int>(sizeof(PktRadio))) break;
            PktRadio p; std::memcpy(&p, pkt, sizeof(p));
            RadioData d;
            std::memcpy(d.channels, p.channels, sizeof(d.channels));
            d.rssi = p.rssi;
            emit radioReceived(d);
            break;
        }
        case PKT_STATUS: {
            if (totalSize < static_cast<int>(sizeof(PktStatus))) break;
            PktStatus p; std::memcpy(&p, pkt, sizeof(p));
            p.state[31] = '\0'; // Ensure null-termination

            // The ESP32 sends a "NO-FC" heartbeat when it thinks the FC is absent.
            // Suppress it when the GCS is actively receiving other packet types
            // (attitude, radio, …): any incoming UDP traffic proves the link is
            // alive, so we must not overwrite the last real status with zeros.
            bool isHeartbeat = (std::string(p.state) == "NO-FC" ||
                                std::string(p.state) == "SPI-ERR");
            if (isHeartbeat && m_hasNonStatusPacket && m_lastNonStatusTimer.elapsed() < 3000)
                break;

            StatusData d;
            d.battery_voltage = p.battery_voltage;
            d.battery_percent = p.battery_percent;
            d.uptime_us       = p.header.timestamp_us;
            d.state = p.state;
            std::memcpy(d.motor_percent, p.motor_percent, 8);
            d.wifi_rssi = p.wifi_rssi;
            emit statusReceived(d);
            break;
        }
        case PKT_PID: {
            if (totalSize < static_cast<int>(sizeof(PktPidValues))) break;
            PktPidValues p; std::memcpy(&p, pkt, sizeof(p));
            PidData d;
            d.rate_roll      = p.rate_roll;
            d.rate_pitch     = p.rate_pitch;
            d.rate_yaw       = p.rate_yaw;
            d.attitude_roll  = p.attitude_roll;
            d.attitude_pitch = p.attitude_pitch;
            d.attitude_yaw   = p.attitude_yaw;
            d.position_x     = p.position_x;
            d.position_y     = p.position_y;
            d.position_z     = p.position_z;
            emit pidReceived(d);
            break;
        }
        case PKT_LOG: {
            if (totalSize < static_cast<int>(sizeof(PktLog))) break;
            PktLog p; std::memcpy(&p, pkt, sizeof(p));
            p.text[127] = '\0'; // Ensure null-termination
            emit logReceived(p.level, QString::fromUtf8(p.text));
            break;
        }
        case PKT_BARO: {
            if (totalSize < static_cast<int>(sizeof(PktBaro))) break;
            PktBaro p; std::memcpy(&p, pkt, sizeof(p));
            BaroData d;
            d.pressure_pa   = p.pressure_pa;
            d.temperature_c = p.temperature_c;
            d.altitude_m    = p.altitude_m;
            emit baroReceived(d);
            break;
        }
        case PKT_CALIB_STATUS: {
            if (totalSize < static_cast<int>(sizeof(PktCalibStatus))) break;
            PktCalibStatus p; std::memcpy(&p, pkt, sizeof(p));
            p.message[63] = '\0'; // Ensure null-termination
            emit calibStatusReceived(p.target, p.status, p.progress,
                                     QString::fromUtf8(p.message));
            break;
        }
        case PKT_FFT: {
            if (totalSize < static_cast<int>(sizeof(PktFft))) break;
            if (hdr->payload_len < static_cast<uint16_t>(sizeof(PktFft)
                                                         - sizeof(PacketHeader)
                                                         - sizeof(uint16_t))) break;
            PktFft p; std::memcpy(&p, pkt, sizeof(p));
            if (p.sensor > 1 || p.axis > 2 || p.bin_count > FFT_BIN_COUNT) break;
            FftData d;
            d.valid              = true;
            d.sensor             = p.sensor;
            d.axis               = p.axis;
            d.bin_count          = p.bin_count;
            d.freq_resolution_hz = p.freq_resolution_hz;
            std::memcpy(d.raw,   p.raw,   sizeof(d.raw));
            std::memcpy(d.notch, p.notch, sizeof(d.notch));
            std::memcpy(d.full,  p.full,  sizeof(d.full));
            emit fftReceived(p.sensor, p.axis, d);
            break;
        }
        case PKT_ACK: {
            if (totalSize < static_cast<int>(sizeof(PktAck))) break;
            PktAck p; std::memcpy(&p, pkt, sizeof(p));
            emit ackReceived(p.ack_type, p.ack_seq, p.success);
            break;
        }
        default:
            AppLogger::warn(QString("PacketParser: unknown packet type 0x%1")
                            .arg(hdr->type, 2, 16, QChar('0')));
            break;
    }

    // Any non-STATUS packet counts as proof that the link is alive
    if (hdr->type != PKT_STATUS)
    {
        if (!m_hasNonStatusPacket)
        {
            m_lastNonStatusTimer.start();
            m_hasNonStatusPacket = true;
        }
        else
        {
            m_lastNonStatusTimer.restart();
        }
    }

    offset += totalSize;
    return true;
}

// ---------------------------------------------------------------------------
// Sequence tracking
// ---------------------------------------------------------------------------
void PacketParser::trackSeq(uint8_t type, uint16_t seq) {
    if (type == 0 || type > 0x0A) return;
    auto& t = m_seqTracker[type];
    if (t.first) {
        t.lastSeq = seq;
        t.first   = false;
        t.received = 1;
        t.expected = 1;
        return;
    }
    uint16_t delta = static_cast<uint16_t>(seq - t.lastSeq);
    t.expected += delta;
    t.received += 1;
    t.lastSeq   = seq;
}

float PacketParser::packetLoss(uint8_t type) const {
    if (type == 0 || type > 0x0A) return 0.0f;
    const auto& t = m_seqTracker[type];
    if (t.expected == 0) return 0.0f;
    float loss = 1.0f - static_cast<float>(t.received) / static_cast<float>(t.expected);
    return qBound(0.0f, loss * 100.0f, 100.0f);
}
