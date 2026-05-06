#include "StatusWidget.h"
#include <QVBoxLayout>
#include <QFormLayout>
#include <QGroupBox>

static QLabel* makeVal() {
    auto* l = new QLabel("--");
    l->setStyleSheet("color: #80c8ff; font-family: monospace; font-size: 13px;");
    return l;
}
static QLabel* makeLbl(const QString& t) {
    auto* l = new QLabel(t);
    l->setStyleSheet("color: #888;");
    return l;
}

StatusWidget::StatusWidget(QWidget* parent) : QWidget(parent) {
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(4, 4, 4, 4);

    auto* box = new QGroupBox("STATUS", this);
    box->setStyleSheet("QGroupBox { color: #aaa; border: 1px solid #444; margin-top: 8px; }"
                       "QGroupBox::title { subcontrol-origin: margin; left: 8px; }");
    auto* form = new QFormLayout(box);
    form->setSpacing(6);
    form->setContentsMargins(8, 4, 8, 4);

    m_voltage = makeVal();
    m_batt    = makeVal();
    m_uptime  = makeVal();

    m_state = new QLabel("--");
    m_state->setStyleSheet("color: #aaaaaa; font-weight: bold; font-size: 15px;");
    m_state->setAlignment(Qt::AlignCenter);

    form->addRow(makeLbl("Voltage:"), m_voltage);
    form->addRow(makeLbl("Battery:"), m_batt);
    form->addRow(makeLbl("Uptime:"),  m_uptime);
    form->addRow(makeLbl("State:"),   m_state);

    outer->addWidget(box);
}

void StatusWidget::updateData(const StatusData& d) {
    m_voltage->setText(QString("%1 V").arg(d.battery_voltage, 0, 'f', 2));
    m_batt   ->setText(QString("%1 %").arg(d.battery_percent));

    uint32_t totalSec = d.uptime_us / 1'000'000u;
    uint32_t h = totalSec / 3600;
    uint32_t m = (totalSec % 3600) / 60;
    uint32_t s = totalSec % 60;
    m_uptime->setText(QString("%1:%2:%3")
        .arg(h, 2, 10, QChar('0'))
        .arg(m, 2, 10, QChar('0'))
        .arg(s, 2, 10, QChar('0')));

    QString state = QString::fromStdString(d.state);
    m_state->setText(state.isEmpty() ? "--" : state);

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
