#pragma once
#include <QWidget>
#include <QLabel>
#include "backend/TelemetryState.h"

class BarometerWidget : public QWidget {
    Q_OBJECT
public:
    explicit BarometerWidget(QWidget* parent = nullptr);
    void updateData(const BaroData& d);

private:
    QLabel* m_pressure    = nullptr;
    QLabel* m_temperature = nullptr;
    QLabel* m_altitude    = nullptr;
};
