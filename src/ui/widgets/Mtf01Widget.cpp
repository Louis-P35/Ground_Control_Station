#include "Mtf01Widget.h"
#include <QVBoxLayout>
#include <QFormLayout>
#include <QGroupBox>

static QLabel* makeValueLabel() {
    auto* l = new QLabel("--");
    l->setStyleSheet("color: #80ff80; font-family: monospace; font-size: 13px;");
    return l;
}

Mtf01Widget::Mtf01Widget(QWidget* parent) : QWidget(parent) {
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(4,4,4,4);

    auto* box = new QGroupBox("MTF-01", this);
    box->setStyleSheet("QGroupBox { color: #aaa; border: 1px solid #444; margin-top: 8px; }"
                       "QGroupBox::title { subcontrol-origin: margin; left: 8px; }");
    auto* form = new QFormLayout(box);
    form->setSpacing(6);

    m_distance = makeValueLabel();
    m_flowX    = makeValueLabel();
    m_flowY    = makeValueLabel();
    m_quality  = makeValueLabel();

    auto labelStyle = [](const QString& s) {
        auto* l = new QLabel(s);
        l->setStyleSheet("color: #888;");
        return l;
    };

    form->addRow(labelStyle("Distance:"), m_distance);
    form->addRow(labelStyle("Flow X:"),   m_flowX);
    form->addRow(labelStyle("Flow Y:"),   m_flowY);
    form->addRow(labelStyle("Quality:"),  m_quality);

    outer->addWidget(box);
}

void Mtf01Widget::updateData(const Mtf01Data& d) {
    m_distance->setText(QString("%1 m").arg(d.distance_m, 0, 'f', 2));
    m_flowX   ->setText(QString("%1 px/s").arg(d.flow_x, 0, 'f', 2));
    m_flowY   ->setText(QString("%1 px/s").arg(d.flow_y, 0, 'f', 2));
    m_quality ->setText(QString("%1 / 255").arg(d.quality));
}
