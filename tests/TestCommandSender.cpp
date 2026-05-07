#include <QTest>
#include <QCoreApplication>
#include <cstring>
#include "TestHelpers.h"
#include "MockUdpLink.h"
#include "backend/CommandSender.h"
#include "backend/Protocol.h"

// ---------------------------------------------------------------------------
// TestCommandSender — unit tests for CommandSender.
//
// Covers: packet structure (magic, version, type, fields, CRC), sequence
// counter increment, ACK-backed retry loop (retry on silence, clear on ACK,
// wrong-seq ACK ignored, max retries then drop), and multiple in-flight
// commands being retried independently.
//
// MockUdpLink captures all outgoing datagrams so tests don't touch the network.
// The retry timer interval is 200 ms; each wait uses 250 ms to give enough margin.
// ---------------------------------------------------------------------------
class TestCommandSender : public QObject {
    Q_OBJECT

private slots:

    // -----------------------------------------------------------------------
    // Packet structure correctness
    // -----------------------------------------------------------------------

    void sendOnce_exactlyOneDatagram() {
        MockUdpLink link;
        CommandSender sender(&link);

        sender.sendSetPid(RATE_ROLL, 1.0f, 0.1f, 0.01f);

        QCOMPARE(link.sent.count(), 1);
        QCOMPARE(link.sent[0].size(), static_cast<int>(sizeof(PktSetPid)));
    }

    void packet_magic() {
        MockUdpLink link;
        CommandSender sender(&link);
        sender.sendSetPid(RATE_ROLL, 1.0f, 0.0f, 0.0f);

        PktSetPid pkt{};
        std::memcpy(&pkt, link.sent[0].constData(), sizeof(pkt));
        QCOMPARE(pkt.header.magic, PACKET_MAGIC);
    }

    void packet_version() {
        MockUdpLink link;
        CommandSender sender(&link);
        sender.sendSetPid(RATE_ROLL, 1.0f, 0.0f, 0.0f);

        PktSetPid pkt{};
        std::memcpy(&pkt, link.sent[0].constData(), sizeof(pkt));
        QCOMPARE(pkt.header.version, PACKET_VERSION);
    }

    void packet_type() {
        MockUdpLink link;
        CommandSender sender(&link);
        sender.sendSetPid(RATE_ROLL, 1.0f, 0.0f, 0.0f);

        PktSetPid pkt{};
        std::memcpy(&pkt, link.sent[0].constData(), sizeof(pkt));
        QCOMPARE(pkt.header.type, static_cast<uint8_t>(PKT_SET_PID));
    }

    void packet_fields() {
        MockUdpLink link;
        CommandSender sender(&link);
        sender.sendSetPid(ATT_PITCH, 1.5f, 0.5f, 0.1f);

        PktSetPid pkt{};
        std::memcpy(&pkt, link.sent[0].constData(), sizeof(pkt));
        QCOMPARE(pkt.axis_id, static_cast<uint8_t>(ATT_PITCH));
        QCOMPARE(pkt.kp, 1.5f);
        QCOMPARE(pkt.ki, 0.5f);
        QCOMPARE(pkt.kd, 0.1f);
    }

    void packet_payloadLen() {
        MockUdpLink link;
        CommandSender sender(&link);
        sender.sendSetPid(RATE_ROLL, 1.0f, 0.0f, 0.0f);

        PktSetPid pkt{};
        std::memcpy(&pkt, link.sent[0].constData(), sizeof(pkt));
        uint16_t expected = sizeof(PktSetPid) - sizeof(PacketHeader) - sizeof(uint16_t);
        QCOMPARE(pkt.header.payload_len, expected);
    }

    void packet_crcValid() {
        MockUdpLink link;
        CommandSender sender(&link);
        sender.sendSetPid(RATE_YAW, 2.0f, 0.2f, 0.02f);

        PktSetPid pkt{};
        std::memcpy(&pkt, link.sent[0].constData(), sizeof(pkt));
        int crcLen = sizeof(PacketHeader) + pkt.header.payload_len;
        uint16_t computed = TestHelpers::crc16(
            reinterpret_cast<const uint8_t*>(&pkt), crcLen);
        QCOMPARE(pkt.crc, computed);
    }

    void seqIncrementsPerSend() {
        MockUdpLink link;
        CommandSender sender(&link);

        sender.sendSetPid(RATE_ROLL,  1.0f, 0.0f, 0.0f);
        sender.sendSetPid(RATE_PITCH, 2.0f, 0.0f, 0.0f);

        QCOMPARE(link.sent.count(), 2);

        PktSetPid p1{}, p2{};
        std::memcpy(&p1, link.sent[0].constData(), sizeof(p1));
        std::memcpy(&p2, link.sent[1].constData(), sizeof(p2));
        QCOMPARE(p1.header.seq, static_cast<uint16_t>(1));
        QCOMPARE(p2.header.seq, static_cast<uint16_t>(2));
    }

    // -----------------------------------------------------------------------
    // ACK-backed retry loop
    // -----------------------------------------------------------------------

    void retryOnNoAck_oneRetryAfter200ms() {
        MockUdpLink link;
        CommandSender sender(&link);

        sender.sendSetPid(RATE_ROLL, 1.0f, 0.0f, 0.0f);
        QCOMPARE(link.sent.count(), 1);

        QTest::qWait(250); // retry timer fires at 200 ms
        QCOMPARE(link.sent.count(), 2);
    }

    void noRetryAfterAck() {
        MockUdpLink link;
        CommandSender sender(&link);

        // First send → seq = 1
        sender.sendSetPid(RATE_ROLL, 1.0f, 0.0f, 0.0f);
        QCOMPARE(link.sent.count(), 1);

        // Feed an ACK for seq=1 directly to the parser (same-thread direct connection)
        PktAck ack{};
        ack.ack_type = PKT_SET_PID;
        ack.ack_seq  = 1;
        ack.success  = 1;
        link.parser()->parse(TestHelpers::buildPacket(PKT_ACK, ack));

        // Wait past the retry interval — pending was cleared, so no retry
        QTest::qWait(250);
        QCOMPARE(link.sent.count(), 1);
    }

    void ackWrongSeq_stillRetries() {
        MockUdpLink link;
        CommandSender sender(&link);

        sender.sendSetPid(RATE_ROLL, 1.0f, 0.0f, 0.0f); // seq=1
        QCOMPARE(link.sent.count(), 1);

        // ACK for seq=99 — does not match, so the command stays pending
        PktAck ack{};
        ack.ack_type = PKT_SET_PID;
        ack.ack_seq  = 99;
        ack.success  = 1;
        link.parser()->parse(TestHelpers::buildPacket(PKT_ACK, ack));

        QTest::qWait(250);
        QCOMPARE(link.sent.count(), 2); // retry happened
    }

    void maxRetries_thenDrop() {
        MockUdpLink link;
        CommandSender sender(&link);

        sender.sendSetPid(RATE_ROLL, 1.0f, 0.0f, 0.0f);
        QCOMPARE(link.sent.count(), 1); // initial send

        // MAX_RETRIES = 3: three retry cycles, each fires after 200 ms
        QTest::qWait(250); QCOMPARE(link.sent.count(), 2); // retry 1
        QTest::qWait(250); QCOMPARE(link.sent.count(), 3); // retry 2
        QTest::qWait(250); QCOMPARE(link.sent.count(), 4); // retry 3

        // Command is now dropped (retriesLeft reached 0) — no more sends
        QTest::qWait(250);
        QCOMPARE(link.sent.count(), 4);
    }

    void multipleCommands_bothRetriedIndependently() {
        MockUdpLink link;
        CommandSender sender(&link);

        sender.sendSetPid(RATE_ROLL,  1.0f, 0.0f, 0.0f); // seq=1
        sender.sendSetPid(RATE_PITCH, 2.0f, 0.0f, 0.0f); // seq=2
        QCOMPARE(link.sent.count(), 2);

        // One retry cycle: both commands are retried
        QTest::qWait(250);
        QCOMPARE(link.sent.count(), 4);
    }

    void ackClearsOnlyMatchingCommand() {
        MockUdpLink link;
        CommandSender sender(&link);

        sender.sendSetPid(RATE_ROLL,  1.0f, 0.0f, 0.0f); // seq=1
        sender.sendSetPid(RATE_PITCH, 2.0f, 0.0f, 0.0f); // seq=2
        QCOMPARE(link.sent.count(), 2);

        // ACK only seq=1 — seq=2 should still retry
        PktAck ack{};
        ack.ack_type = PKT_SET_PID;
        ack.ack_seq  = 1;
        ack.success  = 1;
        link.parser()->parse(TestHelpers::buildPacket(PKT_ACK, ack));

        QTest::qWait(250);
        // Only seq=2 was retried (1 more datagram)
        QCOMPARE(link.sent.count(), 3);
    }
};

int TestCommandSender_run(int argc, char** argv) {
    TestCommandSender t;
    return QTest::qExec(&t, argc, argv);
}

#include "TestCommandSender.moc"
