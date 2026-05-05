#pragma once
#include <QWidget>

// ---------------------------------------------------------------------------
// CompassWidget — 2D circular compass.
// A rotating dial shows the current heading; numeric value shown in the center.
// ---------------------------------------------------------------------------

class CompassWidget : public QWidget {
    Q_OBJECT
public:
    explicit CompassWidget(QWidget* parent = nullptr);
    void setHeading(float degrees);

protected:
    void paintEvent(QPaintEvent*) override;

private:
    float m_heading = 0.0f; // degrees, 0 = North
};
