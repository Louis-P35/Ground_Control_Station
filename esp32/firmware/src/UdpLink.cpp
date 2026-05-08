#include "UdpLink.h"
#include <cstring>
#include <Arduino.h>

// ---------------------------------------------------------------------------
// begin
// ---------------------------------------------------------------------------

void UdpLink::begin(uint16_t localPort, const char* gcsIp, uint16_t gcsPort) {
    strncpy(m_gcsIp, gcsIp, sizeof(m_gcsIp) - 1);
    m_gcsPort = gcsPort;
    m_udp.begin(localPort);
    Serial.printf("[UDP] Listening on :%u  →  GCS %s:%u\n", localPort, gcsIp, gcsPort);
}

// ---------------------------------------------------------------------------
// update — TX then RX
// ---------------------------------------------------------------------------

void UdpLink::update(SpiSlave& spi) {
    // ── TX: forward any fresh SPI data as GCS UDP packets ───────────────────
    if (spi.newAttitude())  sendAttitude(spi.attitude());
    if (spi.newStatus())    sendStatus  (spi.status());
    if (spi.newPid())       sendPid     (spi.pid());
    if (spi.newCalib())     sendCalib   (spi.calib());
    if (spi.newLog())       sendLog     (spi.log());

    // ── RX: receive commands from GCS and forward to FC ─────────────────────
    receiveCommands(spi);
}

// ---------------------------------------------------------------------------
// TX helpers — build a GCS-compatible UDP packet and send it
// ---------------------------------------------------------------------------

void UdpLink::fillHeader(PacketHeader& hdr, uint8_t type, uint16_t payloadLen) {
    hdr.magic        = PACKET_MAGIC;
    hdr.version      = PACKET_VERSION;
    hdr.type         = type;
    hdr.timestamp_us = static_cast<uint32_t>(millis() * 1000UL);
    hdr.seq          = nextSeq(type);
    hdr.payload_len  = payloadLen;
}

void UdpLink::sendAttitude(const SpiPayloadAttitude& d) {
    PktAttitude pkt{};
    fillHeader(pkt.header, PKT_ATTITUDE,
               sizeof(PktAttitude) - sizeof(PacketHeader) - sizeof(uint16_t));
    pkt.qw = d.qw; pkt.qx = d.qx; pkt.qy = d.qy; pkt.qz = d.qz;
    pkt.gx = d.gx; pkt.gy = d.gy; pkt.gz = d.gz;
    pkt.ax = d.ax; pkt.ay = d.ay; pkt.az = d.az;
    pkt.crc = crc16(reinterpret_cast<const uint8_t*>(&pkt),
                    sizeof(PacketHeader) + pkt.header.payload_len);
    sendPacket(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

void UdpLink::sendStatus(const SpiPayloadStatus& d) {
    PktStatus pkt{};
    fillHeader(pkt.header, PKT_STATUS,
               sizeof(PktStatus) - sizeof(PacketHeader) - sizeof(uint16_t));
    pkt.battery_voltage  = d.battery_voltage;
    pkt.battery_current  = d.battery_current;
    pkt.battery_percent  = d.battery_percent;
    memcpy(pkt.state, d.state, sizeof(pkt.state));
    pkt.state[sizeof(pkt.state) - 1] = '\0';
    memcpy(pkt.motor_percent, d.motor_percent, sizeof(pkt.motor_percent));
    pkt.wifi_rssi = d.wifi_rssi;
    pkt.crc = crc16(reinterpret_cast<const uint8_t*>(&pkt),
                    sizeof(PacketHeader) + pkt.header.payload_len);
    sendPacket(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

void UdpLink::sendPid(const SpiPayloadPid& d) {
    PktPidValues pkt{};
    fillHeader(pkt.header, PKT_PID,
               sizeof(PktPidValues) - sizeof(PacketHeader) - sizeof(uint16_t));
    pkt.rate_roll     = d.rate_roll;   pkt.rate_pitch    = d.rate_pitch;
    pkt.rate_yaw      = d.rate_yaw;    pkt.attitude_roll  = d.att_roll;
    pkt.attitude_pitch = d.att_pitch;  pkt.attitude_yaw   = d.att_yaw;
    pkt.position_x    = d.pos_x;       pkt.position_y     = d.pos_y;
    pkt.position_z    = d.pos_z;
    pkt.crc = crc16(reinterpret_cast<const uint8_t*>(&pkt),
                    sizeof(PacketHeader) + pkt.header.payload_len);
    sendPacket(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

void UdpLink::sendCalib(const SpiPayloadCalibStatus& d) {
    PktCalibStatus pkt{};
    fillHeader(pkt.header, PKT_CALIB_STATUS,
               sizeof(PktCalibStatus) - sizeof(PacketHeader) - sizeof(uint16_t));
    pkt.target   = d.target;
    pkt.status   = d.status;
    pkt.progress = d.progress;
    memcpy(pkt.message, d.message, sizeof(pkt.message));
    pkt.message[sizeof(pkt.message) - 1] = '\0';
    pkt.crc = crc16(reinterpret_cast<const uint8_t*>(&pkt),
                    sizeof(PacketHeader) + pkt.header.payload_len);
    sendPacket(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

void UdpLink::sendLog(const SpiPayloadLog& d) {
    PktLog pkt{};
    fillHeader(pkt.header, PKT_LOG,
               sizeof(PktLog) - sizeof(PacketHeader) - sizeof(uint16_t));
    pkt.level = d.level;
    memcpy(pkt.text, d.text, sizeof(pkt.text));
    pkt.text[sizeof(pkt.text) - 1] = '\0';
    pkt.crc = crc16(reinterpret_cast<const uint8_t*>(&pkt),
                    sizeof(PacketHeader) + pkt.header.payload_len);
    sendPacket(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

void UdpLink::sendAck(uint8_t ackType, uint16_t ackSeq, uint8_t success) {
    PktAck pkt{};
    fillHeader(pkt.header, PKT_ACK,
               sizeof(PktAck) - sizeof(PacketHeader) - sizeof(uint16_t));
    pkt.ack_type = ackType;
    pkt.ack_seq  = ackSeq;
    pkt.success  = success;
    pkt.crc = crc16(reinterpret_cast<const uint8_t*>(&pkt),
                    sizeof(PacketHeader) + pkt.header.payload_len);
    sendPacket(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

void UdpLink::sendPacket(const uint8_t* data, size_t len) {
    m_udp.beginPacket(m_gcsIp, m_gcsPort);
    m_udp.write(data, static_cast<int>(len));
    m_udp.endPacket();
}

// ---------------------------------------------------------------------------
// RX — receive one pending UDP datagram per call, validate, and dispatch
// ---------------------------------------------------------------------------

void UdpLink::receiveCommands(SpiSlave& spi) {
    int pktSize = m_udp.parsePacket();
    if (pktSize <= 0) return;

    uint8_t buf[256];
    int len = m_udp.read(buf, sizeof(buf));
    if (len < static_cast<int>(sizeof(PacketHeader) + sizeof(uint16_t)))
        return;

    // Validate magic and version
    PacketHeader hdr{};
    memcpy(&hdr, buf, sizeof(hdr));
    if (hdr.magic != PACKET_MAGIC || hdr.version != PACKET_VERSION) return;

    // Validate CRC
    int bodyLen = static_cast<int>(sizeof(PacketHeader)) + hdr.payload_len;
    if (bodyLen + static_cast<int>(sizeof(uint16_t)) > len) return;

    uint16_t receivedCrc;
    memcpy(&receivedCrc, buf + bodyLen, sizeof(uint16_t));
    if (crc16(buf, static_cast<size_t>(bodyLen)) != receivedCrc) return;

    switch (hdr.type) {
        case PKT_SET_PID:   handleSetPid  (buf, static_cast<size_t>(len), spi); break;
        case PKT_CALIB_CMD: handleCalibCmd(buf, static_cast<size_t>(len), spi); break;
        default: break; // unknown command type — ignore silently
    }
}

void UdpLink::handleSetPid(const uint8_t* buf, size_t len, SpiSlave& spi) {
    if (len < sizeof(PktSetPid)) return;

    // Forward raw packet bytes (header + payload + CRC) to the FC via SPI MISO
    spi.setPendingCommand(PKT_SET_PID, buf, sizeof(PktSetPid));

    // Immediate ACK to the GCS so it stops retrying
    PacketHeader hdr{};
    memcpy(&hdr, buf, sizeof(hdr));
    sendAck(PKT_SET_PID, hdr.seq, 1 /*success*/);
}

void UdpLink::handleCalibCmd(const uint8_t* buf, size_t len, SpiSlave& spi) {
    if (len < sizeof(PktCalibCmd)) return;

    spi.setPendingCommand(PKT_CALIB_CMD, buf, sizeof(PktCalibCmd));

    PacketHeader hdr{};
    memcpy(&hdr, buf, sizeof(hdr));
    sendAck(PKT_CALIB_CMD, hdr.seq, 1 /*success*/);
}

// ---------------------------------------------------------------------------
// crc16 — CRC-16/CCITT (same polynomial as GCS CommandSender and Protocol.h)
// ---------------------------------------------------------------------------

uint16_t UdpLink::crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}
