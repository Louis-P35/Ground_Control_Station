#pragma once
#include <QMutex>
#include <QMutexLocker>
#include "Protocol.h"

// ---------------------------------------------------------------------------
// TelemetryState — thread-safe snapshot of the latest drone state.
// Updated by the network thread, read by the UI thread via copy.
// ---------------------------------------------------------------------------

struct AttitudeData {
    float qw = 1, qx = 0, qy = 0, qz = 0;
    float gx = 0, gy = 0, gz = 0;
    float ax = 0, ay = 0, az = 0;
};

struct GpsData {
    double  latitude    = 0;
    double  longitude   = 0;
    float   altitude_m  = 0;
    float   speed_ms    = 0;
    float   heading_deg = 0;
    uint8_t satellites  = 0;
    uint8_t fix_type    = 0;
};

struct Mtf01Data {
    float   distance_m = 0;
    float   flow_x     = 0;
    float   flow_y     = 0;
    uint8_t quality    = 0;
};

struct RadioData {
    uint16_t channels[8] = {};
    uint8_t  rssi        = 0;
};

struct StatusData {
    float   battery_voltage  = 0;
    float   battery_current  = 0;
    uint8_t battery_percent  = 0;
    uint8_t armed            = 0;
    uint8_t flight_mode      = 0;
    uint8_t motor_percent[4] = {};
    uint8_t wifi_rssi        = 0;
};

struct PidData {
    PidAxis rate_roll     = {};
    PidAxis rate_pitch    = {};
    PidAxis rate_yaw      = {};
    PidAxis attitude_roll = {};
    PidAxis attitude_pitch= {};
    PidAxis attitude_yaw  = {};
    PidAxis position_x    = {};
    PidAxis position_y    = {};
    PidAxis position_z    = {};
};

class TelemetryState {
public:
    // Called from network thread
    void updateAttitude(const AttitudeData& d) { QMutexLocker l(&m_mutex); m_attitude = d; }
    void updateGps     (const GpsData&      d) { QMutexLocker l(&m_mutex); m_gps      = d; }
    void updateMtf01   (const Mtf01Data&    d) { QMutexLocker l(&m_mutex); m_mtf01    = d; }
    void updateRadio   (const RadioData&    d) { QMutexLocker l(&m_mutex); m_radio    = d; }
    void updateStatus  (const StatusData&   d) { QMutexLocker l(&m_mutex); m_status   = d; }
    void updatePid     (const PidData&      d) { QMutexLocker l(&m_mutex); m_pid      = d; }

    // Called from UI thread — returns a copy
    AttitudeData attitude() const { QMutexLocker l(&m_mutex); return m_attitude; }
    GpsData      gps()      const { QMutexLocker l(&m_mutex); return m_gps;      }
    Mtf01Data    mtf01()    const { QMutexLocker l(&m_mutex); return m_mtf01;    }
    RadioData    radio()    const { QMutexLocker l(&m_mutex); return m_radio;    }
    StatusData   status()   const { QMutexLocker l(&m_mutex); return m_status;   }
    PidData      pid()      const { QMutexLocker l(&m_mutex); return m_pid;      }

private:
    mutable QMutex m_mutex;
    AttitudeData   m_attitude;
    GpsData        m_gps;
    Mtf01Data      m_mtf01;
    RadioData      m_radio;
    StatusData     m_status;
    PidData        m_pid;
};
