#include <QTest>
#include "backend/TelemetryState.h"

// ---------------------------------------------------------------------------
// TestTelemetryState — unit tests for TelemetryState.
//
// Verifies that each update*() method stores data correctly and that the
// corresponding getter returns an identical copy. Also checks default values
// to confirm the struct is zero-initialised.
// ---------------------------------------------------------------------------
class TestTelemetryState : public QObject {
    Q_OBJECT

private slots:

    // -----------------------------------------------------------------------
    // Default values — state is safe to read before any update
    // -----------------------------------------------------------------------

    void defaults_attitude() {
        TelemetryState s;
        auto d = s.attitude();
        // Identity quaternion: qw=1, rest=0
        QCOMPARE(d.qw, 1.0f);
        QCOMPARE(d.qx, 0.0f);
        QCOMPARE(d.qy, 0.0f);
        QCOMPARE(d.qz, 0.0f);
        QCOMPARE(d.gx, 0.0f);
        QCOMPARE(d.ax, 0.0f);
    }

    void defaults_gps() {
        TelemetryState s;
        auto d = s.gps();
        QCOMPARE(d.latitude,   0.0);
        QCOMPARE(d.longitude,  0.0);
        QCOMPARE(d.altitude_m, 0.0f);
        QCOMPARE(d.satellites, static_cast<uint8_t>(0));
        QCOMPARE(d.fix_type,   static_cast<uint8_t>(0));
    }

    // -----------------------------------------------------------------------
    // Round-trip: update then read returns identical data
    // -----------------------------------------------------------------------

    void updateAttitude_roundTrip() {
        TelemetryState s;
        AttitudeData in;
        in.qw = 0.7071f; in.qx = 0.7071f; in.qy = 0.0f; in.qz = 0.0f;
        in.gx = 5.0f;    in.gy = -3.0f;   in.gz = 1.0f;
        in.ax = 0.1f;    in.ay = 0.2f;    in.az = -9.8f;
        s.updateAttitude(in);
        auto out = s.attitude();
        QCOMPARE(out.qw, in.qw);
        QCOMPARE(out.qx, in.qx);
        QCOMPARE(out.gy, in.gy);
        QCOMPARE(out.az, in.az);
    }

    void updateGps_roundTrip() {
        TelemetryState s;
        GpsData in;
        in.latitude    = 43.2965;
        in.longitude   = 5.3811;
        in.altitude_m  = 75.0f;
        in.speed_ms    = 10.0f;
        in.heading_deg = 180.0f;
        in.satellites  = 12;
        in.fix_type    = 2;
        s.updateGps(in);
        auto out = s.gps();
        QCOMPARE(out.latitude,    in.latitude);
        QCOMPARE(out.longitude,   in.longitude);
        QCOMPARE(out.altitude_m,  in.altitude_m);
        QCOMPARE(out.speed_ms,    in.speed_ms);
        QCOMPARE(out.heading_deg, in.heading_deg);
        QCOMPARE(out.satellites,  in.satellites);
        QCOMPARE(out.fix_type,    in.fix_type);
    }

    void updateMtf01_roundTrip() {
        TelemetryState s;
        Mtf01Data in;
        in.distance_m = 3.14f;
        in.flow_x     = 8.0f;
        in.flow_y     = -4.0f;
        in.quality    = 220;
        s.updateMtf01(in);
        auto out = s.mtf01();
        QCOMPARE(out.distance_m, in.distance_m);
        QCOMPARE(out.flow_x,     in.flow_x);
        QCOMPARE(out.flow_y,     in.flow_y);
        QCOMPARE(out.quality,    in.quality);
    }

    void updateRadio_roundTrip() {
        TelemetryState s;
        RadioData in;
        for (int i = 0; i < 8; ++i) in.channels[i] = static_cast<uint16_t>(1100 + i * 50);
        in.rssi = 95;
        s.updateRadio(in);
        auto out = s.radio();
        for (int i = 0; i < 8; ++i)
            QCOMPARE(out.channels[i], in.channels[i]);
        QCOMPARE(out.rssi, in.rssi);
    }

    void updateStatus_roundTrip() {
        TelemetryState s;
        StatusData in;
        in.battery_voltage = 12.6f;
        in.battery_percent = 90;
        in.state           = "ARMED";
        for (int i = 0; i < 8; ++i) in.motor_percent[i] = static_cast<uint8_t>(i * 10);
        in.wifi_rssi = 70;
        in.uptime_us = 123456;
        s.updateStatus(in);
        auto out = s.status();
        QCOMPARE(out.battery_voltage, in.battery_voltage);
        QCOMPARE(out.battery_percent, in.battery_percent);
        QCOMPARE(out.state,           in.state);
        QCOMPARE(out.wifi_rssi,       in.wifi_rssi);
        QCOMPARE(out.uptime_us,       in.uptime_us);
        for (int i = 0; i < 8; ++i)
            QCOMPARE(out.motor_percent[i], in.motor_percent[i]);
    }

    void updatePid_roundTrip() {
        TelemetryState s;
        PidData in;
        in.rate_roll      = {1.0f, 0.1f, 0.01f};
        in.rate_pitch     = {1.5f, 0.15f, 0.015f};
        in.rate_yaw       = {2.0f, 0.2f, 0.02f};
        in.attitude_roll  = {0.3f, 0.03f, 0.003f};
        in.attitude_pitch = {0.4f, 0.04f, 0.004f};
        in.attitude_yaw   = {0.5f, 0.05f, 0.005f};
        in.position_x     = {0.6f, 0.06f, 0.006f};
        in.position_y     = {0.7f, 0.07f, 0.007f};
        in.position_z     = {0.8f, 0.08f, 0.008f};
        s.updatePid(in);
        auto out = s.pid();
        QCOMPARE(out.rate_roll.kp,     in.rate_roll.kp);
        QCOMPARE(out.rate_roll.ki,     in.rate_roll.ki);
        QCOMPARE(out.rate_roll.kd,     in.rate_roll.kd);
        QCOMPARE(out.attitude_yaw.kp,  in.attitude_yaw.kp);
        QCOMPARE(out.position_z.kd,    in.position_z.kd);
    }

    void updateOverwritesPreviousValue() {
        TelemetryState s;
        GpsData first;
        first.latitude = 10.0;
        s.updateGps(first);

        GpsData second;
        second.latitude = 20.0;
        s.updateGps(second);

        QCOMPARE(s.gps().latitude, 20.0);
    }
};

int TestTelemetryState_run(int argc, char** argv) {
    TestTelemetryState t;
    return QTest::qExec(&t, argc, argv);
}

#include "TestTelemetryState.moc"
