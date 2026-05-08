#include "BarometerWidget.h"
#include <QVBoxLayout>
#include <QFormLayout>
#include <QGroupBox>

static QLabel* makeValueLabel() {
    auto* l = new QLabel("--");
    l->setStyleSheet("color: #80d0ff; font-family: monospace; font-size: 13px;");
    return l;
}

BarometerWidget::BarometerWidget(QWidget* parent) : QWidget(parent) {
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(4, 4, 4, 4);

    auto* box = new QGroupBox("Barometer", this);
    box->setStyleSheet("QGroupBox { color: #aaa; border: 1px solid #444; margin-top: 8px; }"
                       "QGroupBox::title { subcontrol-origin: margin; left: 8px; }");
    auto* form = new QFormLayout(box);
    form->setSpacing(6);

    m_pressure    = makeValueLabel();
    m_temperature = makeValueLabel();
    m_altitude    = makeValueLabel();

    auto labelStyle = [](const QString& s) {
        auto* l = new QLabel(s);
        l->setStyleSheet("color: #888;");
        return l;
    };

    form->addRow(labelStyle("Pressure:"),    m_pressure);
    form->addRow(labelStyle("Temperature:"), m_temperature);
    form->addRow(labelStyle("Altitude:"),    m_altitude);

    outer->addWidget(box);
    outer->addStretch(1);
}

void BarometerWidget::updateData(const BaroData& d) {
    // Convert Pa to hPa for readability (1 hPa = 100 Pa)
    m_pressure   ->setText(QString("%1 hPa").arg(d.pressure_pa / 100.0f, 0, 'f', 1));
    m_temperature->setText(QString("%1 °C").arg(d.temperature_c, 0, 'f', 1));
    m_altitude   ->setText(QString("%1 m").arg(d.altitude_m, 0, 'f', 1));
}
