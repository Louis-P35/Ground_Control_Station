#include "PidConfigWidget.h"
#include <QGridLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QPushButton>
#include <QLabel>
#include <QFormLayout>

static QLineEdit* makePidEdit() {
    auto* e = new QLineEdit("0.000");
    e->setMaximumWidth(70);
    e->setStyleSheet("background: #222; color: #adf; border: 1px solid #444;");
    return e;
}

PidConfigWidget::PidConfigWidget(QWidget* parent) : QWidget(parent) {
    auto* mainLayout = new QHBoxLayout(this);
    mainLayout->setContentsMargins(4,4,4,4);
    mainLayout->setSpacing(8);

    // ---- PID groups ----
    auto* pidArea = new QWidget(this);
    auto* pidGrid = new QGridLayout(pidArea);
    pidGrid->setSpacing(4);
    pidGrid->setContentsMargins(0,0,0,0);

    // Header
    auto headerStyle = [](const QString& t) {
        auto* l = new QLabel(t);
        l->setStyleSheet("color: #aaa; font-weight: bold;");
        l->setAlignment(Qt::AlignCenter);
        return l;
    };
    pidGrid->addWidget(headerStyle("Axis"), 0, 0);
    pidGrid->addWidget(headerStyle("Kp"),   0, 1);
    pidGrid->addWidget(headerStyle("Ki"),   0, 2);
    pidGrid->addWidget(headerStyle("Kd"),   0, 3);

    // Rate group (rows 1–4): RATE_ROLL=0, RATE_PITCH=1, RATE_YAW=2
    addGroupSection(pidGrid, "Rate", 0, 1,
                    {"Roll","Pitch","Yaw"}, "Send Rate", RATE_ROLL);
    // Attitude group (rows 5–8): ATT_ROLL=3, ATT_PITCH=4, ATT_YAW=5
    addGroupSection(pidGrid, "Attitude", 3, 5,
                    {"Roll","Pitch","Yaw"}, "Send Att.", ATT_ROLL);
    // Position group (rows 9–12): POS_X=6, POS_Y=7, POS_Z=8
    addGroupSection(pidGrid, "Position", 6, 9,
                    {"X","Y","Z"}, "Send Pos.", POS_X);

    mainLayout->addWidget(pidArea, 3);

    // ---- Status panel ----
    auto* statusBox = new QGroupBox("Status", this);
    statusBox->setStyleSheet("QGroupBox { color: #aaa; border: 1px solid #444; margin-top: 8px; }"
                             "QGroupBox::title { subcontrol-origin: margin; left: 8px; }");
    auto* sf = new QFormLayout(statusBox);
    sf->setSpacing(8);

    auto mkVal = [](const QString& init = "--") {
        auto* l = new QLabel(init);
        l->setStyleSheet("color: #80c8ff; font-family: monospace;");
        return l;
    };
    auto mkLbl = [](const QString& t) {
        auto* l = new QLabel(t);
        l->setStyleSheet("color: #888;");
        return l;
    };

    m_voltage = mkVal();
    m_current = mkVal();
    m_batt    = mkVal();

    // FSM state — large centered label, color changes with state
    m_state = new QLabel("--");
    m_state->setStyleSheet("color: #aaaaaa; font-weight: bold; font-size: 15px;");
    m_state->setAlignment(Qt::AlignCenter);

    sf->addRow(mkLbl("Voltage:"), m_voltage);
    sf->addRow(mkLbl("Current:"), m_current);
    sf->addRow(mkLbl("Battery:"), m_batt);
    sf->addRow(mkLbl("State:"),   m_state);

    mainLayout->addWidget(statusBox, 1);
}

void PidConfigWidget::addGroupSection(QGridLayout* grid, const QString& title,
                                      int startAxis, int startRow,
                                      const QStringList& axisNames,
                                      const QString& sendLabel,
                                      int firstAxisEnum)
{
    // Section title spanning all columns
    auto* titleLbl = new QLabel(title);
    titleLbl->setStyleSheet("color: #ffcc44; font-weight: bold; background: #2a2a2a;");
    titleLbl->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    grid->addWidget(titleLbl, startRow, 0, 1, 4);

    for (int i = 0; i < 3; ++i) {
        int row = startRow + 1 + i;
        int axisIdx = startAxis + i;

        auto* nameLbl = new QLabel(axisNames[i]);
        nameLbl->setStyleSheet("color: #ccc;");
        grid->addWidget(nameLbl, row, 0);

        m_axes[axisIdx].kp = makePidEdit();
        m_axes[axisIdx].ki = makePidEdit();
        m_axes[axisIdx].kd = makePidEdit();
        grid->addWidget(m_axes[axisIdx].kp, row, 1);
        grid->addWidget(m_axes[axisIdx].ki, row, 2);
        grid->addWidget(m_axes[axisIdx].kd, row, 3);
    }

    // Send button after the 3 axis rows
    auto* sendBtn = new QPushButton(sendLabel);
    sendBtn->setStyleSheet("QPushButton { background: #1a4a8a; color: white; border: 1px solid #2a6ac0; padding: 4px; }"
                           "QPushButton:hover { background: #2a6ac0; }");
    int btnRow = startRow + 4;
    grid->addWidget(sendBtn, btnRow, 0, 1, 4);

    // Capture for lambda
    int capturedStart  = startAxis;
    int capturedEnum   = firstAxisEnum;
    connect(sendBtn, &QPushButton::clicked, this, [this, capturedStart, capturedEnum](){
        for (int i = 0; i < 3; ++i) {
            int axisIdx = capturedStart + i;
            float kp = m_axes[axisIdx].kp->text().toFloat();
            float ki = m_axes[axisIdx].ki->text().toFloat();
            float kd = m_axes[axisIdx].kd->text().toFloat();
            emit sendPidRequested(static_cast<PidAxisId>(capturedEnum + i), kp, ki, kd);
        }
    });
}

void PidConfigWidget::updatePid(const PidData& d) {
    auto set = [](AxisRow& row, const PidAxis& a) {
        row.kp->setText(QString::number(a.kp, 'f', 4));
        row.ki->setText(QString::number(a.ki, 'f', 4));
        row.kd->setText(QString::number(a.kd, 'f', 4));
    };
    set(m_axes[0], d.rate_roll);
    set(m_axes[1], d.rate_pitch);
    set(m_axes[2], d.rate_yaw);
    set(m_axes[3], d.attitude_roll);
    set(m_axes[4], d.attitude_pitch);
    set(m_axes[5], d.attitude_yaw);
    set(m_axes[6], d.position_x);
    set(m_axes[7], d.position_y);
    set(m_axes[8], d.position_z);
}

void PidConfigWidget::updateStatus(const StatusData& d) {
    m_voltage->setText(QString("%1 V").arg(d.battery_voltage, 0, 'f', 2));
    m_current->setText(QString("%1 A").arg(d.battery_current, 0, 'f', 2));
    m_batt   ->setText(QString("%1 %").arg(d.battery_percent));

    QString state = QString::fromStdString(d.state);
    m_state->setText(state.isEmpty() ? "--" : state);

    // Color hint: red for states that include "DISARM" or "ERROR", green for "ARM"/"FLY", gray otherwise
    QString upper = state.toUpper();
    QString color;
    if (upper.contains("ERROR") || upper.contains("FAULT") || upper.contains("DISARM"))
        color = "#ff4444";
    else if (upper.contains("ARM") || upper.contains("FLY") || upper.contains("LAND"))
        color = "#44ff44";
    else
        color = "#ffcc44";

    m_state->setStyleSheet(QString("color: %1; font-weight: bold; font-size: 15px;").arg(color));
}
