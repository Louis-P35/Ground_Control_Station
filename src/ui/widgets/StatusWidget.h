#pragma once
#include <QWidget>
#include <QLabel>
#include "backend/TelemetryState.h"

// ---------------------------------------------------------------------------
// StatusWidget — displays drone status info: battery, uptime, FSM state.
// Standalone widget placed independently in the main layout.
// ---------------------------------------------------------------------------

class StatusWidget : public QWidget {
    Q_OBJECT
public:
    explicit StatusWidget(QWidget* parent = nullptr);
    void updateData(const StatusData& d);

private:
    QLabel* m_voltage = nullptr;
    QLabel* m_batt    = nullptr;
    QLabel* m_uptime  = nullptr;
    QLabel* m_state   = nullptr;
};
