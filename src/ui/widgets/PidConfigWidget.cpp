#include "PidConfigWidget.h"
#include <QGridLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QPushButton>
#include <QLabel>

static QLineEdit* makePidEdit() {
    auto* e = new QLineEdit("0.000");
    e->setMaximumWidth(65);
    e->setFixedHeight(20);
    e->setStyleSheet("background: #222; color: #adf; border: 1px solid #444; padding: 1px;");
    return e;
}

PidConfigWidget::PidConfigWidget(QWidget* parent) : QWidget(parent) {
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(4, 4, 4, 4);
    outer->setSpacing(2);

    auto* box = new QGroupBox("PID Configuration", this);
    box->setStyleSheet("QGroupBox { color: #aaa; border: 1px solid #444; margin-top: 8px; }"
                       "QGroupBox::title { subcontrol-origin: margin; left: 8px; }");
    auto* pidGrid = new QGridLayout(box);
    pidGrid->setSpacing(2);
    pidGrid->setContentsMargins(4, 4, 4, 4);

    // Column headers
    auto header = [](const QString& t) {
        auto* l = new QLabel(t);
        l->setStyleSheet("color: #aaa; font-weight: bold;");
        l->setAlignment(Qt::AlignCenter);
        return l;
    };
    pidGrid->addWidget(header("Axis"), 0, 0);
    pidGrid->addWidget(header("Kp"),   0, 1);
    pidGrid->addWidget(header("Ki"),   0, 2);
    pidGrid->addWidget(header("Kd"),   0, 3);

    // Each section uses 5 rows: title + 3 axes + send button.
    // Row 0: column headers. Sections start at rows 1, 6, 11.
    addGroupSection(pidGrid, "Rate",     0, 1,  {"Roll","Pitch","Yaw"}, "Send Rate", RATE_ROLL);
    addGroupSection(pidGrid, "Attitude", 3, 6,  {"Roll","Pitch","Yaw"}, "Send Att.", ATT_ROLL);
    addGroupSection(pidGrid, "Position", 6, 11, {"X","Y","Z"},          "Send Pos.", POS_X);

    outer->addWidget(box);
}

void PidConfigWidget::addGroupSection(QGridLayout* grid, const QString& title,
                                      int startAxis, int startRow,
                                      const QStringList& axisNames,
                                      const QString& sendLabel,
                                      int firstAxisEnum)
{
    auto* titleLbl = new QLabel(title);
    titleLbl->setStyleSheet("color: #ffcc44; font-weight: bold; background: #2a2a2a; padding: 1px 4px;");
    titleLbl->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    titleLbl->setFixedHeight(16);
    grid->addWidget(titleLbl, startRow, 0, 1, 4);

    for (int i = 0; i < 3; ++i) {
        int row     = startRow + 1 + i;
        int axisIdx = startAxis + i;

        auto* nameLbl = new QLabel(axisNames[i]);
        nameLbl->setStyleSheet("color: #ccc; font-size: 11px;");
        nameLbl->setFixedHeight(18);
        grid->addWidget(nameLbl, row, 0);

        m_axes[axisIdx].kp = makePidEdit();
        m_axes[axisIdx].ki = makePidEdit();
        m_axes[axisIdx].kd = makePidEdit();
        grid->addWidget(m_axes[axisIdx].kp, row, 1);
        grid->addWidget(m_axes[axisIdx].ki, row, 2);
        grid->addWidget(m_axes[axisIdx].kd, row, 3);
    }

    auto* sendBtn = new QPushButton(sendLabel);
    sendBtn->setFixedHeight(20);
    sendBtn->setStyleSheet("QPushButton { background: #1a4a8a; color: white; border: 1px solid #2a6ac0; padding: 2px; font-size: 11px; }"
                           "QPushButton:hover { background: #2a6ac0; }");
    grid->addWidget(sendBtn, startRow + 4, 0, 1, 4);

    int capturedStart = startAxis;
    int capturedEnum  = firstAxisEnum;
    connect(sendBtn, &QPushButton::clicked, this, [this, capturedStart, capturedEnum]() {
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
    set(m_axes[0], d.rate_roll);    set(m_axes[1], d.rate_pitch); set(m_axes[2], d.rate_yaw);
    set(m_axes[3], d.attitude_roll);set(m_axes[4], d.attitude_pitch);set(m_axes[5], d.attitude_yaw);
    set(m_axes[6], d.position_x);  set(m_axes[7], d.position_y); set(m_axes[8], d.position_z);
}
