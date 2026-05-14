#pragma once
#include <QMutex>
#include <QMutexLocker>
#include <string>
#include "Protocol.h" // shared with ESP32 firmware — lives in common/

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
    float       battery_voltage  = 0;
    uint8_t     battery_percent  = 0;
    std::string state;            // FSM state string received from drone
    uint8_t     motor_percent[8] = {};
    uint8_t     wifi_rssi        = 0;
    uint32_t    uptime_us        = 0; // Drone uptime from packet header timestamp
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

struct BaroData {
    float pressure_pa   = 101325.0f; // Standard sea-level pressure in Pascals
    float temperature_c = 0.0f;      // Celsius
    float altitude_m    = 0.0f;      // Barometric altitude in meters
};

struct CalibStatusData {
    uint8_t     status   = CALIB_IDLE; // CalibStatusValue
    uint8_t     progress = 0;          // 0-100 %
    std::string message;               // Human-readable status from drone
};

// Three parallel spectra for one sensor+axis combination.
// Indexed by [sensor][axis] where sensor ∈ {FFT_SENSOR_ACCEL, FFT_SENSOR_GYRO}
// and axis ∈ {FFT_AXIS_X, FFT_AXIS_Y, FFT_AXIS_Z}.
struct FftData
{
    bool     valid              = false;
    uint8_t  sensor             = 0;
    uint8_t  axis               = 0;
    uint16_t bin_count          = 0;
    float    freq_resolution_hz = 0.0f; // Hz per bin
    float    raw  [FFT_BIN_COUNT]  = {};
    float    notch[FFT_BIN_COUNT]  = {};
    float    full [FFT_BIN_COUNT]  = {};
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
    void updateBaro       (const BaroData&       d) { QMutexLocker l(&m_mutex); m_baro = d; }
    void updateCalibStatus(uint8_t target, const CalibStatusData& d) {
        if (target > 2) return;
        QMutexLocker l(&m_mutex);
        m_calibStatus[target] = d;
    }
    void updateFft(uint8_t sensor, uint8_t axis, const FftData& d) {
        if (sensor > 1 || axis > 2) return;
        QMutexLocker l(&m_mutex);
        m_fft[sensor][axis] = d;
    }

    // Called from UI thread — returns a copy
    AttitudeData   attitude()                const { QMutexLocker l(&m_mutex); return m_attitude; }
    GpsData        gps()                     const { QMutexLocker l(&m_mutex); return m_gps;      }
    Mtf01Data      mtf01()                   const { QMutexLocker l(&m_mutex); return m_mtf01;    }
    RadioData      radio()                   const { QMutexLocker l(&m_mutex); return m_radio;    }
    StatusData     status()                  const { QMutexLocker l(&m_mutex); return m_status;   }
    PidData        pid()                     const { QMutexLocker l(&m_mutex); return m_pid;      }
    BaroData       baro()                    const { QMutexLocker l(&m_mutex); return m_baro;     }
    CalibStatusData calibStatus(uint8_t target) const {
        if (target > 2) return {};
        QMutexLocker l(&m_mutex);
        return m_calibStatus[target];
    }
    FftData fft(uint8_t sensor, uint8_t axis) const {
        if (sensor > 1 || axis > 2) return {};
        QMutexLocker l(&m_mutex);
        return m_fft[sensor][axis];
    }

private:
    mutable QMutex  m_mutex;
    AttitudeData    m_attitude;
    GpsData         m_gps;
    Mtf01Data       m_mtf01;
    RadioData       m_radio;
    StatusData      m_status;
    PidData         m_pid;
    BaroData        m_baro;
    CalibStatusData m_calibStatus[3]; // indexed by CalibTarget (0=accel, 1=mag, 2=level)
    FftData         m_fft[2][3];      // indexed by [sensor][axis]
};
