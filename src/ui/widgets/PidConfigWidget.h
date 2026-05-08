#pragma once
#include <QWidget>
#include <QLineEdit>
#include <QGridLayout>
#include <QStringList>
#include <array>
#include "backend/TelemetryState.h"
#include "Protocol.h"

// ---------------------------------------------------------------------------
// PidConfigWidget — editable PID fields only (Rate / Attitude / Position).
// Status info is handled by StatusWidget.
// ---------------------------------------------------------------------------

class PidConfigWidget : public QWidget {
    Q_OBJECT
public:
    explicit PidConfigWidget(QWidget* parent = nullptr);
    void updatePid(const PidData& d);

signals:
    void sendPidRequested(PidAxisId axis, float kp, float ki, float kd);

private:
    struct AxisRow {
        QLineEdit* kp = nullptr;
        QLineEdit* ki = nullptr;
        QLineEdit* kd = nullptr;
    };
    std::array<AxisRow, 9> m_axes;

    void addGroupSection(QGridLayout* grid, const QString& title,
                         int startAxis, int startRow,
                         const QStringList& axisNames,
                         const QString& sendLabel,
                         int firstAxisEnum);
};
