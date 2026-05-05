#include "GpsWidget.h"
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

GpsWidget::GpsWidget(QWidget* parent) : QWidget(parent) {
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(4,4,4,4);

    auto* box = new QGroupBox("GPS", this);
    box->setStyleSheet("QGroupBox { color: #aaa; border: 1px solid #444; margin-top: 8px; }"
                       "QGroupBox::title { subcontrol-origin: margin; left: 8px; }");
    auto* form = new QFormLayout(box);
    form->setSpacing(5);

    m_lat     = makeVal(); m_lon     = makeVal();
    m_alt     = makeVal(); m_speed   = makeVal();
    m_heading = makeVal(); m_sats    = makeVal();
    m_fixType = makeVal();

    form->addRow(makeLbl("Latitude:"),  m_lat);
    form->addRow(makeLbl("Longitude:"), m_lon);
    form->addRow(makeLbl("Altitude:"),  m_alt);
    form->addRow(makeLbl("Speed:"),     m_speed);
    form->addRow(makeLbl("Heading:"),   m_heading);
    form->addRow(makeLbl("Satellites:"),m_sats);
    form->addRow(makeLbl("Fix:"),       m_fixType);

    outer->addWidget(box);
}

void GpsWidget::updateData(const GpsData& d) {
    m_lat    ->setText(QString::number(d.latitude,    'f', 6) + "°");
    m_lon    ->setText(QString::number(d.longitude,   'f', 6) + "°");
    m_alt    ->setText(QString("%1 m").arg(d.altitude_m,  0, 'f', 1));
    m_speed  ->setText(QString("%1 m/s").arg(d.speed_ms, 0, 'f', 1));
    m_heading->setText(QString("%1°").arg(d.heading_deg, 0, 'f', 1));
    m_sats   ->setText(QString::number(d.satellites));

    // Fix type with color indicator
    static const char* fixNames[] = { "None", "2D", "3D" };
    static const char* fixColors[]= { "#ff4444", "#ff9922", "#44ff44" };
    uint8_t ft = qMin<uint8_t>(d.fix_type, 2);
    m_fixType->setText(fixNames[ft]);
    m_fixType->setStyleSheet(QString("color: %1; font-family: monospace; font-size: 13px; font-weight: bold;")
                              .arg(fixColors[ft]));
}
