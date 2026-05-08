#pragma once
#include <WiFiUdp.h>
#include <cstdint>
#include "Protocol.h"
#include "SpiSlave.h"

// ---------------------------------------------------------------------------
// UdpLink — WiFi UDP transmitter and receiver.
//
// TX: reads fresh telemetry from SpiSlave and sends the corresponding GCS
//     UDP packets (same binary format as Protocol.h).
//
// RX: polls for incoming UDP datagrams from the GCS, validates them, and
//     forwards valid commands to SpiSlave::setPendingCommand().
//     Immediately sends PKT_ACK back to the GCS for each accepted command.
//
// Usage in loop():
//   udpLink.update(spiSlave);
// ---------------------------------------------------------------------------

class UdpLink {
public:
    void begin(uint16_t localPort, const char* gcsIp, uint16_t gcsPort);

    // Call every loop() iteration
    void update(SpiSlave& spi);

private:
    // ── TX ────────────────────────────────────────────────────────────────────
    void sendAttitude (const SpiPayloadAttitude&    d);
    void sendStatus   (const SpiPayloadStatus&      d);
    void sendPid      (const SpiPayloadPid&         d);
    void sendCalib    (const SpiPayloadCalibStatus& d);
    void sendLog      (const SpiPayloadLog&         d);
    void sendAck      (uint8_t ackType, uint16_t ackSeq, uint8_t success);

    void    sendPacket(const uint8_t* data, size_t len);
    uint16_t nextSeq(uint8_t type) { return ++m_seqs[type]; }

    // ── RX ────────────────────────────────────────────────────────────────────
    void receiveCommands(SpiSlave& spi);
    void handleSetPid  (const uint8_t* buf, size_t len, SpiSlave& spi);
    void handleCalibCmd(const uint8_t* buf, size_t len, SpiSlave& spi);

    // ── Shared ────────────────────────────────────────────────────────────────
    static uint16_t crc16(const uint8_t* data, size_t len);
    void fillHeader(PacketHeader& hdr, uint8_t type, uint16_t payloadLen);

    WiFiUDP  m_udp;
    char     m_gcsIp[40] = {};
    uint16_t m_gcsPort   = 0;

    // One sequence counter per packet type (indexed by type byte)
    uint16_t m_seqs[0x21] = {};
};
