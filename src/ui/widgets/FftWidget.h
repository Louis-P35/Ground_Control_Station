#pragma once
#include <QWidget>
#include "backend/TelemetryState.h"

class QPushButton;
class QWidget;

// ---------------------------------------------------------------------------
// FftWidget — full-tab FFT spectrum viewer.
//
// Displays three superimposed spectra (raw, notch-filtered, notch+pass-filtered)
// for the currently selected sensor (accelerometer or gyroscope) and axis (X/Y/Z).
//
// Data is pushed via updateFft() each time a PKT_FFT packet is received.
// The plot is dB-scaled on the Y-axis and auto-ranges to the peak of the raw spectrum.
// ---------------------------------------------------------------------------

class FftWidget : public QWidget
{
    Q_OBJECT
public:
    explicit FftWidget(QWidget* parent = nullptr);

    // Called from MainWindow whenever a PKT_FFT packet is received.
    void updateFft(uint8_t sensor, uint8_t axis, const FftData& data);

protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    void setSensor(uint8_t sensor);
    void setAxis(uint8_t axis);
    void updateButtonStyles();

    // Currently displayed combination
    uint8_t m_sensor = FFT_SENSOR_GYRO;
    uint8_t m_axis   = FFT_AXIS_X;

    // Cached spectra — one entry per sensor+axis combination
    FftData m_data[2][3];

    // Control bar (child widget, pinned to top)
    QWidget*     m_ctrlBar  = nullptr;
    QPushButton* m_accelBtn = nullptr;
    QPushButton* m_gyroBtn  = nullptr;
    QPushButton* m_axisX    = nullptr;
    QPushButton* m_axisY    = nullptr;
    QPushButton* m_axisZ    = nullptr;

    static constexpr int CTRL_BAR_H = 42;
};
