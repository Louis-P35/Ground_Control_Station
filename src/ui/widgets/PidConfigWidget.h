#pragma once
#include <QWidget>
#include <QLineEdit>
#include <QLabel>
#include <QGridLayout>
#include <QStringList>
#include <array>
#include "backend/TelemetryState.h"
#include "backend/Protocol.h"

// ---------------------------------------------------------------------------
// PidConfigWidget — editable PID fields + read-only status fields.
// Emits sendPidRequested when the user clicks a Send button.
// ---------------------------------------------------------------------------

class PidConfigWidget : public QWidget {
    Q_OBJECT
public:
    explicit PidConfigWidget(QWidget* parent = nullptr);

    void updatePid(const PidData& d);
    void updateStatus(const StatusData& d);

signals:
    void sendPidRequested(PidAxisId axis, float kp, float ki, float kd);

private:
    struct AxisRow {
        QLineEdit* kp = nullptr;
        QLineEdit* ki = nullptr;
        QLineEdit* kd = nullptr;
    };

    // 9 axes: rate(3) + attitude(3) + position(3)
    std::array<AxisRow, 9> m_axes;

    // Status
    QLabel* m_voltage  = nullptr;
    QLabel* m_current  = nullptr;
    QLabel* m_batt     = nullptr;
    QLabel* m_armed    = nullptr;
    QLabel* m_mode     = nullptr;

    void addGroupSection(QGridLayout* grid, const QString& title,
                         int startAxis, int startRow,
                         const QStringList& axisNames,
                         const QString& sendLabel,
                         int firstAxisEnum);
};
