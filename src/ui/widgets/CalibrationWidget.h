#pragma once
#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QProgressBar>
#include "Protocol.h"

// ---------------------------------------------------------------------------
// CalibrationWidget — UI for on-ground sensor calibration.
//
// Displays one panel per calibration target (accelerometer, magnetometer,
// level attitude). Each panel shows a description, live progress, status
// indicator, and action buttons. Emits calibCmdRequested when the user
// presses a button; the main window forwards this to CommandSender.
// ---------------------------------------------------------------------------

class CalibrationWidget : public QWidget {
    Q_OBJECT
public:
    explicit CalibrationWidget(QWidget* parent = nullptr);

    // Called from the UI thread when a PKT_CALIB_STATUS packet arrives.
    void updateCalibStatus(uint8_t target, uint8_t status,
                           uint8_t progress, const QString& message);

signals:
    void calibCmdRequested(uint8_t target, uint8_t action);

private:
    // Per-target panel references
    struct Panel {
        QLabel*       statusDot  = nullptr; // Colored status indicator
        QLabel*       statusText = nullptr; // "Idle" / "Running" / …
        QLabel*       msgLabel   = nullptr; // Drone status message
        QProgressBar* progress   = nullptr; // nullptr for LEVEL
        QPushButton*  startBtn   = nullptr;
        QPushButton*  stopBtn    = nullptr; // nullptr for LEVEL
    };
    Panel m_panels[3]; // indexed by CalibTarget

    QWidget* buildPanel(uint8_t target,
                        const QString& title,
                        const QString& description,
                        bool           hasProgress);

    static void applyStatus(Panel& p, uint8_t status, uint8_t progress,
                            const QString& message);
};
