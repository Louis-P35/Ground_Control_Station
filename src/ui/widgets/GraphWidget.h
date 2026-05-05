#pragma once
#include <QWidget>
#include <QCheckBox>
#include <deque>
#include <array>
#include "backend/TelemetryState.h"

// ---------------------------------------------------------------------------
// GraphWidget — rolling real-time graph of up to 8 configurable curves.
// X-axis: last N seconds (default 10 s). Y-axis: auto-scale.
// ---------------------------------------------------------------------------

class GraphWidget : public QWidget {
    Q_OBJECT
public:
    explicit GraphWidget(QWidget* parent = nullptr);

    void pushAttitude(const AttitudeData& d);
    void pushGps     (const GpsData& d);
    void pushMtf01   (const Mtf01Data& d);

protected:
    void paintEvent(QPaintEvent*) override;
    void timerEvent(QTimerEvent*) override;

private:
    static constexpr int   NUM_CURVES  = 8;
    static constexpr int   WINDOW_SECS = 10;
    static constexpr float SAMPLE_RATE = 100.0f; // Hz (attitude)
    static constexpr int   MAX_SAMPLES = static_cast<int>(WINDOW_SECS * SAMPLE_RATE);

    struct Sample { float t; float v; }; // t = elapsed seconds
    std::array<std::deque<Sample>, NUM_CURVES> m_curves;
    std::array<QCheckBox*, NUM_CURVES>         m_checks{};
    std::array<QColor, NUM_CURVES>             m_colors;
    std::array<QString, NUM_CURVES>            m_labels;

    float m_elapsed = 0.0f; // seconds since start
    int   m_timerId = 0;

    // Latest raw values pushed by network callbacks
    float m_roll = 0, m_pitch = 0, m_yaw = 0;
    float m_altM = 0;
    float m_gyrX = 0, m_gyrY = 0, m_gyrZ = 0;
    float m_flowQ = 0;

    void sampleNow();
    void drawGrid(QPainter& p, QRectF plotRect, float yMin, float yMax) const;

    // Euler angles from quaternion
    static void quatToEuler(float qw, float qx, float qy, float qz,
                            float& roll, float& pitch, float& yaw);
};
