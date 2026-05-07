#include "CommandSender.h"
#include "AppLogger.h"
#include <cstring>

static constexpr int RETRY_INTERVAL_MS = 200;
static constexpr int MAX_RETRIES       = 3;

CommandSender::CommandSender(UdpLink* link, QObject* parent)
    : QObject(parent), m_link(link)
{
    connect(link->parser(), &PacketParser::ackReceived,
            this,           &CommandSender::onAckReceived);

    m_retryTimer = new QTimer(this);
    m_retryTimer->setInterval(RETRY_INTERVAL_MS);
    connect(m_retryTimer, &QTimer::timeout, this, &CommandSender::onRetryTimer);
}

void CommandSender::sendSetPid(PidAxisId axis, float kp, float ki, float kd) {
    uint16_t seq = ++m_seqCounter;
    QByteArray data = buildSetPid(axis, kp, ki, kd, seq);

    PendingCmd cmd;
    cmd.data        = data;
    cmd.retriesLeft = MAX_RETRIES;
    cmd.seq         = seq;
    m_pending[seq]  = cmd;

    AppLogger::info(QString("CommandSender: PID sent axis=%1 kp=%2 ki=%3 kd=%4 seq=%5")
                    .arg(static_cast<int>(axis))
                    .arg(kp, 0, 'f', 4).arg(ki, 0, 'f', 4).arg(kd, 0, 'f', 4)
                    .arg(seq));
    m_link->sendDatagram(data);

    if (!m_retryTimer->isActive())
        m_retryTimer->start();
}

void CommandSender::onRetryTimer() {
    if (m_pending.isEmpty()) {
        m_retryTimer->stop();
        return;
    }

    QList<uint16_t> toRemove;
    for (auto it = m_pending.begin(); it != m_pending.end(); ++it) {
        auto& cmd = it.value();
        if (cmd.retriesLeft <= 0) {
            AppLogger::warn(QString("CommandSender: PID command seq=%1 dropped (max retries exhausted)")
                            .arg(it.key()));
            toRemove.append(it.key());
            continue;
        }
        m_link->sendDatagram(cmd.data);
        --cmd.retriesLeft;
    }
    for (uint16_t seq : toRemove)
        m_pending.remove(seq);
}

void CommandSender::onAckReceived(uint8_t ackType, uint16_t ackSeq, uint8_t /*success*/) {
    if (ackType == PKT_SET_PID)
        m_pending.remove(ackSeq);
}

// ---------------------------------------------------------------------------
// Packet builder
// ---------------------------------------------------------------------------
uint16_t CommandSender::crc16(const uint8_t* data, int len) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

QByteArray CommandSender::buildSetPid(PidAxisId axis, float kp, float ki, float kd, uint16_t seq) {
    PktSetPid pkt{};
    pkt.header.magic       = PACKET_MAGIC;
    pkt.header.version     = PACKET_VERSION;
    pkt.header.type        = PKT_SET_PID;
    pkt.header.timestamp_us = 0; // GCS doesn't have drone time
    pkt.header.seq         = seq;
    pkt.header.payload_len = sizeof(PktSetPid) - sizeof(PacketHeader) - sizeof(uint16_t);
    pkt.axis_id = static_cast<uint8_t>(axis);
    pkt.kp = kp; pkt.ki = ki; pkt.kd = kd;

    int crcLen = sizeof(PacketHeader) + pkt.header.payload_len;
    pkt.crc = crc16(reinterpret_cast<const uint8_t*>(&pkt), crcLen);

    QByteArray out(sizeof(PktSetPid), Qt::Uninitialized);
    std::memcpy(out.data(), &pkt, sizeof(PktSetPid));
    return out;
}
