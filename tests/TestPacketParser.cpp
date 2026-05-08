#include <QTest>
#include <QSignalSpy>
#include <cstring>
#include "TestHelpers.h"
#include "backend/PacketParser.h"
#include "backend/Protocol.h"

// ---------------------------------------------------------------------------
// TestPacketParser — unit tests for PacketParser.
//
// Covers: valid decode of all packet types, malformed datagrams (bad magic,
// bad version, CRC mismatch, truncated header/payload, under-declared
// payload_len), multi-packet datagrams, junk prefix recovery,
// per-type packet-loss tracking, and null-termination safety.
// ---------------------------------------------------------------------------
class TestPacketParser : public QObject {
    Q_OBJECT

private slots:

    void initTestCase() {
        qRegisterMetaType<AttitudeData>();
        qRegisterMetaType<GpsData>();
        qRegisterMetaType<Mtf01Data>();
        qRegisterMetaType<RadioData>();
        qRegisterMetaType<StatusData>();
        qRegisterMetaType<PidData>();
        qRegisterMetaType<BaroData>();
    }

    // -----------------------------------------------------------------------
    // Valid decode — all fields round-trip correctly
    // -----------------------------------------------------------------------

    void attitudeValid() {
        PacketParser parser;
        QSignalSpy spy(&parser, &PacketParser::attitudeReceived);

        PktAttitude p{};
        p.qw = 1.0f; p.qx = 0.1f; p.qy = 0.2f; p.qz = 0.3f;
        p.gx = 10.f; p.gy = 20.f; p.gz = 30.f;
        p.ax = -9.81f; p.ay = 0.5f; p.az = 0.1f;
        parser.parse(TestHelpers::buildPacket(PKT_ATTITUDE, p));

        QCOMPARE(spy.count(), 1);
        auto d = spy.at(0).at(0).value<AttitudeData>();
        QCOMPARE(d.qw, 1.0f);
        QCOMPARE(d.qx, 0.1f);
        QCOMPARE(d.qy, 0.2f);
        QCOMPARE(d.qz, 0.3f);
        QCOMPARE(d.gx, 10.f);
        QCOMPARE(d.gy, 20.f);
        QCOMPARE(d.gz, 30.f);
        QCOMPARE(d.ax, -9.81f);
        QCOMPARE(d.ay, 0.5f);
        QCOMPARE(d.az, 0.1f);
    }

    void gpsValid() {
        PacketParser parser;
        QSignalSpy spy(&parser, &PacketParser::gpsReceived);

        PktGps p{};
        p.latitude    = 48.8566;
        p.longitude   = 2.3522;
        p.altitude_m  = 150.0f;
        p.speed_ms    = 5.5f;
        p.heading_deg = 270.0f;
        p.satellites  = 8;
        p.fix_type    = 2;
        parser.parse(TestHelpers::buildPacket(PKT_GPS, p));

        QCOMPARE(spy.count(), 1);
        auto d = spy.at(0).at(0).value<GpsData>();
        QCOMPARE(d.latitude,    48.8566);
        QCOMPARE(d.longitude,   2.3522);
        QCOMPARE(d.altitude_m,  150.0f);
        QCOMPARE(d.speed_ms,    5.5f);
        QCOMPARE(d.heading_deg, 270.0f);
        QCOMPARE(d.satellites,  static_cast<uint8_t>(8));
        QCOMPARE(d.fix_type,    static_cast<uint8_t>(2));
    }

    void mtf01Valid() {
        PacketParser parser;
        QSignalSpy spy(&parser, &PacketParser::mtf01Received);

        PktMtf01 p{};
        p.distance_m = 2.5f;
        p.flow_x     = 10.0f;
        p.flow_y     = -5.0f;
        p.quality    = 200;
        parser.parse(TestHelpers::buildPacket(PKT_MTF01, p));

        QCOMPARE(spy.count(), 1);
        auto d = spy.at(0).at(0).value<Mtf01Data>();
        QCOMPARE(d.distance_m, 2.5f);
        QCOMPARE(d.flow_x,     10.0f);
        QCOMPARE(d.flow_y,     -5.0f);
        QCOMPARE(d.quality,    static_cast<uint8_t>(200));
    }

    void radioValid() {
        PacketParser parser;
        QSignalSpy spy(&parser, &PacketParser::radioReceived);

        PktRadio p{};
        for (int i = 0; i < 8; ++i)
            p.channels[i] = static_cast<uint16_t>(1000 + i * 100);
        p.rssi = 80;
        parser.parse(TestHelpers::buildPacket(PKT_RADIO, p));

        QCOMPARE(spy.count(), 1);
        auto d = spy.at(0).at(0).value<RadioData>();
        for (int i = 0; i < 8; ++i)
            QCOMPARE(d.channels[i], static_cast<uint16_t>(1000 + i * 100));
        QCOMPARE(d.rssi, static_cast<uint8_t>(80));
    }

    void statusValid() {
        PacketParser parser;
        QSignalSpy spy(&parser, &PacketParser::statusReceived);

        PktStatus p{};
        p.battery_voltage = 11.4f;
        p.battery_percent = 75;
        std::strncpy(p.state, "FLYING", sizeof(p.state));
        for (int i = 0; i < 8; ++i) p.motor_percent[i] = static_cast<uint8_t>(50 + i);
        p.wifi_rssi = 90;
        parser.parse(TestHelpers::buildPacket(PKT_STATUS, p));

        QCOMPARE(spy.count(), 1);
        auto d = spy.at(0).at(0).value<StatusData>();
        QCOMPARE(d.battery_voltage, 11.4f);
        QCOMPARE(d.battery_percent, static_cast<uint8_t>(75));
        QCOMPARE(d.state,           std::string("FLYING"));
        QCOMPARE(d.wifi_rssi,       static_cast<uint8_t>(90));
        for (int i = 0; i < 8; ++i)
            QCOMPARE(d.motor_percent[i], static_cast<uint8_t>(50 + i));
    }

    void pidValid() {
        PacketParser parser;
        QSignalSpy spy(&parser, &PacketParser::pidReceived);

        PktPidValues p{};
        p.rate_roll     = {1.0f, 0.1f, 0.01f};
        p.rate_pitch    = {1.1f, 0.11f, 0.011f};
        p.rate_yaw      = {1.2f, 0.12f, 0.012f};
        p.attitude_roll = {0.5f, 0.05f, 0.005f};
        p.position_z    = {3.0f, 0.3f, 0.03f};
        parser.parse(TestHelpers::buildPacket(PKT_PID, p));

        QCOMPARE(spy.count(), 1);
        auto d = spy.at(0).at(0).value<PidData>();
        QCOMPARE(d.rate_roll.kp,     1.0f);
        QCOMPARE(d.rate_roll.ki,     0.1f);
        QCOMPARE(d.rate_roll.kd,     0.01f);
        QCOMPARE(d.rate_pitch.kp,    1.1f);
        QCOMPARE(d.rate_yaw.kp,      1.2f);
        QCOMPARE(d.attitude_roll.kp, 0.5f);
        QCOMPARE(d.position_z.kp,    3.0f);
    }

    void baroValid() {
        PacketParser parser;
        QSignalSpy spy(&parser, &PacketParser::baroReceived);

        PktBaro p{};
        p.pressure_pa   = 101325.0f;
        p.temperature_c = 22.5f;
        p.altitude_m    = 52.3f;
        parser.parse(TestHelpers::buildPacket(PKT_BARO, p));

        QCOMPARE(spy.count(), 1);
        auto d = spy.at(0).at(0).value<BaroData>();
        QCOMPARE(d.pressure_pa,   101325.0f);
        QCOMPARE(d.temperature_c, 22.5f);
        QCOMPARE(d.altitude_m,    52.3f);
    }

    void logValid() {
        PacketParser parser;
        QSignalSpy spy(&parser, &PacketParser::logReceived);

        PktLog p{};
        p.level = 2; // WARN
        std::strncpy(p.text, "Motor overheat", sizeof(p.text));
        parser.parse(TestHelpers::buildPacket(PKT_LOG, p));

        QCOMPARE(spy.count(), 1);
        QCOMPARE(spy.at(0).at(0).toUInt(), 2u);
        QCOMPARE(spy.at(0).at(1).toString(), QString("Motor overheat"));
    }

    void ackValid() {
        PacketParser parser;
        QSignalSpy spy(&parser, &PacketParser::ackReceived);

        PktAck p{};
        p.ack_type = PKT_SET_PID;
        p.ack_seq  = 42;
        p.success  = 1;
        parser.parse(TestHelpers::buildPacket(PKT_ACK, p));

        QCOMPARE(spy.count(), 1);
        QCOMPARE(spy.at(0).at(0).toUInt(), static_cast<uint>(PKT_SET_PID));
        QCOMPARE(spy.at(0).at(1).toUInt(), 42u);
        QCOMPARE(spy.at(0).at(2).toUInt(), 1u);
    }

    // -----------------------------------------------------------------------
    // Malformed inputs — no signal, no crash
    // -----------------------------------------------------------------------

    void emptyDatagram_noSignal() {
        PacketParser parser;
        QSignalSpy spy(&parser, &PacketParser::attitudeReceived);
        parser.parse(QByteArray());
        QCOMPARE(spy.count(), 0);
    }

    void junkBytes_noSignal() {
        PacketParser parser;
        QSignalSpy spy(&parser, &PacketParser::attitudeReceived);
        // 0xFF bytes: as uint16_t = 0xFFFF ≠ PACKET_MAGIC (0xABCD)
        parser.parse(QByteArray(100, static_cast<char>(0xFF)));
        QCOMPARE(spy.count(), 0);
    }

    void badMagic_noSignal() {
        PacketParser parser;
        QSignalSpy spy(&parser, &PacketParser::attitudeReceived);

        QByteArray pkt = TestHelpers::buildPacket(PKT_ATTITUDE, PktAttitude{});
        pkt[0] = 0x00;
        pkt[1] = 0x00; // corrupt magic
        parser.parse(pkt);

        QCOMPARE(spy.count(), 0);
    }

    void badVersion_noSignal() {
        PacketParser parser;
        QSignalSpy spy(&parser, &PacketParser::attitudeReceived);

        QByteArray pkt = TestHelpers::buildPacket(PKT_ATTITUDE, PktAttitude{});
        pkt[2] = 0xFF; // version field is byte 2
        parser.parse(pkt);

        QCOMPARE(spy.count(), 0);
    }

    void crcMismatch_noSignal() {
        PacketParser parser;
        QSignalSpy spy(&parser, &PacketParser::attitudeReceived);

        PktAttitude p{};
        p.qw = 1.0f;
        QByteArray pkt = TestHelpers::buildPacket(PKT_ATTITUDE, p);
        pkt[pkt.size() - 1] ^= 0x01; // flip one bit in the CRC
        parser.parse(pkt);

        QCOMPARE(spy.count(), 0);
    }

    void truncatedHeader_noSignal() {
        PacketParser parser;
        QSignalSpy spy(&parser, &PacketParser::attitudeReceived);

        QByteArray pkt = TestHelpers::buildPacket(PKT_ATTITUDE, PktAttitude{});
        pkt.resize(static_cast<int>(sizeof(PacketHeader)) - 4);
        parser.parse(pkt);

        QCOMPARE(spy.count(), 0);
    }

    void truncatedPayload_noSignal() {
        PacketParser parser;
        QSignalSpy spy(&parser, &PacketParser::attitudeReceived);

        // Header declares full payload_len but we remove the last 10 bytes.
        // Parser sees: available < totalSize → returns false → no signal.
        QByteArray pkt = TestHelpers::buildPacket(PKT_ATTITUDE, PktAttitude{});
        pkt.resize(pkt.size() - 10);
        parser.parse(pkt);

        QCOMPARE(spy.count(), 0);
    }

    void payloadLenTooSmall_noSignal() {
        PacketParser parser;
        QSignalSpy spy(&parser, &PacketParser::attitudeReceived);

        // payload_len = 2: CRC is valid, but totalSize < sizeof(PktAttitude),
        // so the per-type size guard inside the switch fires and no signal is emitted.
        QByteArray pkt = TestHelpers::buildShortPacket(PKT_ATTITUDE, 2);
        parser.parse(pkt);

        QCOMPARE(spy.count(), 0);
    }

    void unknownPacketType_noSignal() {
        PacketParser parser;
        QSignalSpy spyAtt(&parser, &PacketParser::attitudeReceived);
        QSignalSpy spyGps(&parser, &PacketParser::gpsReceived);

        // Type 0xAA hits the default: branch; no signal is emitted
        QByteArray pkt = TestHelpers::buildPacket(static_cast<uint8_t>(0xAA), PktAttitude{});
        parser.parse(pkt);

        QCOMPARE(spyAtt.count(), 0);
        QCOMPARE(spyGps.count(), 0);
    }

    // -----------------------------------------------------------------------
    // Multi-packet datagrams
    // -----------------------------------------------------------------------

    void twoPacketsSameType_twoSignals() {
        PacketParser parser;
        QSignalSpy spy(&parser, &PacketParser::attitudeReceived);

        PktAttitude p1{};
        p1.qw = 1.0f;
        PktAttitude p2{};
        p2.qw = 0.5f;

        QByteArray datagram;
        datagram += TestHelpers::buildPacket(PKT_ATTITUDE, p1, 1);
        datagram += TestHelpers::buildPacket(PKT_ATTITUDE, p2, 2);
        parser.parse(datagram);

        QCOMPARE(spy.count(), 2);
        QCOMPARE(spy.at(0).at(0).value<AttitudeData>().qw, 1.0f);
        QCOMPARE(spy.at(1).at(0).value<AttitudeData>().qw, 0.5f);
    }

    void attitudeThenGps_bothSignalsInOrder() {
        PacketParser parser;
        QSignalSpy spyAtt(&parser, &PacketParser::attitudeReceived);
        QSignalSpy spyGps(&parser, &PacketParser::gpsReceived);

        PktAttitude a{};
        a.qw = 1.0f;
        PktGps g{};
        g.latitude = 48.8566;

        QByteArray datagram;
        datagram += TestHelpers::buildPacket(PKT_ATTITUDE, a, 1);
        datagram += TestHelpers::buildPacket(PKT_GPS, g, 1);
        parser.parse(datagram);

        QCOMPARE(spyAtt.count(), 1);
        QCOMPARE(spyGps.count(), 1);
        QCOMPARE(spyAtt.at(0).at(0).value<AttitudeData>().qw, 1.0f);
        QCOMPARE(spyGps.at(0).at(0).value<GpsData>().latitude, 48.8566);
    }

    void junkThenValidPacket_parsesValid() {
        PacketParser parser;
        QSignalSpy spy(&parser, &PacketParser::attitudeReceived);

        // 0x01 bytes cannot accidentally form PACKET_MAGIC (0xABCD / bytes 0xCD 0xAB)
        QByteArray datagram(10, static_cast<char>(0x01));

        PktAttitude p{};
        p.qw = 1.0f;
        datagram += TestHelpers::buildPacket(PKT_ATTITUDE, p, 1);
        parser.parse(datagram);

        QCOMPARE(spy.count(), 1);
        QCOMPARE(spy.at(0).at(0).value<AttitudeData>().qw, 1.0f);
    }

    // -----------------------------------------------------------------------
    // Packet-loss tracking
    // -----------------------------------------------------------------------

    void packetLoss_noGap_zeroPercent() {
        PacketParser parser;

        PktAttitude p{};
        for (uint16_t seq = 1; seq <= 10; ++seq)
            parser.parse(TestHelpers::buildPacket(PKT_ATTITUDE, p, seq));

        QCOMPARE(parser.packetLoss(PKT_ATTITUDE), 0.0f);
    }

    void packetLoss_oneGap_nonZero() {
        PacketParser parser;

        PktAttitude p{};
        parser.parse(TestHelpers::buildPacket(PKT_ATTITUDE, p, 1));
        // seq 2 missing
        parser.parse(TestHelpers::buildPacket(PKT_ATTITUDE, p, 3));

        // expected=3, received=2 → loss ≈ 33.3%
        float loss = parser.packetLoss(PKT_ATTITUDE);
        QVERIFY2(loss > 0.0f,   "Expected non-zero packet loss");
        QVERIFY2(loss < 100.0f, "Packet loss should not be 100%");
    }

    void packetLoss_largeGap_highPercent() {
        PacketParser parser;

        PktAttitude p{};
        parser.parse(TestHelpers::buildPacket(PKT_ATTITUDE, p, 1));
        parser.parse(TestHelpers::buildPacket(PKT_ATTITUDE, p, 101));

        // expected=101, received=2 → loss = 1 - 2/101 ≈ 98%
        QVERIFY2(parser.packetLoss(PKT_ATTITUDE) > 90.0f,
                 "Expected high packet loss for large gap");
    }

    void packetLoss_perTypeIsIndependent() {
        PacketParser parser;

        PktAttitude a{};
        PktGps g{};
        // Attitude: consecutive → 0% loss
        parser.parse(TestHelpers::buildPacket(PKT_ATTITUDE, a, 1));
        parser.parse(TestHelpers::buildPacket(PKT_ATTITUDE, a, 2));
        // GPS: skip seq 2 and 3 → some loss
        parser.parse(TestHelpers::buildPacket(PKT_GPS, g, 1));
        parser.parse(TestHelpers::buildPacket(PKT_GPS, g, 4));

        QCOMPARE(parser.packetLoss(PKT_ATTITUDE), 0.0f);
        QVERIFY2(parser.packetLoss(PKT_GPS) > 0.0f, "GPS should show packet loss");
    }

    void sequenceWraparound_zeroLoss() {
        PacketParser parser;

        // uint16_t arithmetic: delta(0 - 65535) = 1 → no gap
        PktAttitude p{};
        parser.parse(TestHelpers::buildPacket(PKT_ATTITUDE, p, 65535));
        parser.parse(TestHelpers::buildPacket(PKT_ATTITUDE, p, 0));

        QCOMPARE(parser.packetLoss(PKT_ATTITUDE), 0.0f);
    }

    void packetLoss_beforeAnyPacket_zeroPercent() {
        PacketParser parser;
        // No packets yet — should return 0, not NaN or garbage
        QCOMPARE(parser.packetLoss(PKT_ATTITUDE), 0.0f);
        QCOMPARE(parser.packetLoss(PKT_GPS),      0.0f);
    }

    // -----------------------------------------------------------------------
    // Null-termination safety
    // -----------------------------------------------------------------------

    void logNoNullTerminator_safe() {
        PacketParser parser;
        QSignalSpy spy(&parser, &PacketParser::logReceived);

        PktLog p{};
        p.level = 0;
        // Fill every byte including the last with 'A' — no null terminator
        std::memset(p.text, 'A', sizeof(p.text));
        parser.parse(TestHelpers::buildPacket(PKT_LOG, p));

        QCOMPARE(spy.count(), 1);
        // Parser forces p.text[127] = '\0' → we get exactly 127 'A' chars
        QString text = spy.at(0).at(1).toString();
        QCOMPARE(text.size(), 127);
        for (QChar c : text)
            QCOMPARE(c, QChar('A'));
    }

    void statusNoNullTerminator_safe() {
        PacketParser parser;
        QSignalSpy spy(&parser, &PacketParser::statusReceived);

        PktStatus p{};
        // Fill entire 32-byte state buffer with 'X' — no null terminator
        std::memset(p.state, 'X', sizeof(p.state));
        parser.parse(TestHelpers::buildPacket(PKT_STATUS, p));

        QCOMPARE(spy.count(), 1);
        // Parser forces p.state[31] = '\0' → state has exactly 31 chars
        auto d = spy.at(0).at(0).value<StatusData>();
        QCOMPARE(static_cast<int>(d.state.size()), 31);
    }
};

int TestPacketParser_run(int argc, char** argv) {
    TestPacketParser t;
    return QTest::qExec(&t, argc, argv);
}

#include "TestPacketParser.moc"
