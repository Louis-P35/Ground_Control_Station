#pragma once
#include <QWidget>
#include "backend/TelemetryState.h"

// ---------------------------------------------------------------------------
// JoystickWidget — two virtual joysticks (Left: Throttle/Yaw, Right: Pitch/Roll)
// Dots move based on received PktRadio channel values.
// Raw channel values and RSSI are shown below.
// ---------------------------------------------------------------------------

class JoystickWidget : public QWidget {
    Q_OBJECT
public:
    explicit JoystickWidget(QWidget* parent = nullptr);
    void updateData(const RadioData& d);

protected:
    void paintEvent(QPaintEvent*) override;

private:
    RadioData m_data;
    void drawStick(QPainter& p, QRectF area, float normX, float normY, const QString& title);
};
