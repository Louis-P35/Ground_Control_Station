#pragma once
#include <QWidget>
#include "backend/TelemetryState.h"

// ---------------------------------------------------------------------------
// MotorWidget — four vertical gauge bars for M1–M4 motor throttle.
// Color: green (low) → yellow (mid) → red (high).
// ---------------------------------------------------------------------------

class MotorWidget : public QWidget {
    Q_OBJECT
public:
    explicit MotorWidget(QWidget* parent = nullptr);
    void updateData(const StatusData& d);

protected:
    void paintEvent(QPaintEvent*) override;

private:
    uint8_t m_motors[4] = {};
};
