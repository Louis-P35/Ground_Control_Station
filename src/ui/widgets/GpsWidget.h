#pragma once
#include <QWidget>
#include <QLabel>
#include "backend/TelemetryState.h"

class GpsWidget : public QWidget {
    Q_OBJECT
public:
    explicit GpsWidget(QWidget* parent = nullptr);
    void updateData(const GpsData& d);

private:
    QLabel* m_lat      = nullptr;
    QLabel* m_lon      = nullptr;
    QLabel* m_alt      = nullptr;
    QLabel* m_speed    = nullptr;
    QLabel* m_heading  = nullptr;
    QLabel* m_sats     = nullptr;
    QLabel* m_fixType  = nullptr;
};
