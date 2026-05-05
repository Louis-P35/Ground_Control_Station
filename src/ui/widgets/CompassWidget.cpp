#include "CompassWidget.h"
#include <QPainter>
#include <QPainterPath>
#include <QtMath>
#include <QGroupBox>
#include <QVBoxLayout>

CompassWidget::CompassWidget(QWidget* parent) : QWidget(parent) {
    setMinimumSize(150, 150);
}

void CompassWidget::setHeading(float degrees) {
    m_heading = degrees;
    update();
}

void CompassWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    int side = qMin(width(), height()) - 20;
    int cx   = width()  / 2;
    int cy   = height() / 2;
    float r  = side / 2.0f;

    // Background circle
    p.setPen(QPen(QColor(80, 80, 80), 2));
    p.setBrush(QColor(20, 20, 20));
    p.drawEllipse(QPointF(cx, cy), r, r);

    // Tick marks and cardinal labels
    p.setPen(Qt::white);
    QFont font = p.font();
    font.setPointSize(8);
    p.setFont(font);
    for (int deg = 0; deg < 360; deg += 10) {
        float a   = qDegreesToRadians(static_cast<float>(deg - m_heading - 90));
        float cos_a = std::cos(a), sin_a = std::sin(a);
        float inner = (deg % 30 == 0) ? r - 14 : r - 8;
        p.drawLine(QPointF(cx + inner  * cos_a, cy + inner  * sin_a),
                   QPointF(cx + (r-2) * cos_a,  cy + (r-2)  * sin_a));
        if (deg % 90 == 0) {
            QString lbl;
            switch (deg) {
                case 0:   lbl = "N"; break;
                case 90:  lbl = "E"; break;
                case 180: lbl = "S"; break;
                case 270: lbl = "W"; break;
            }
            float lx = cx + (r-26) * cos_a;
            float ly = cy + (r-26) * sin_a;
            p.drawText(QRectF(lx-10, ly-10, 20, 20), Qt::AlignCenter, lbl);
        }
    }

    // North needle (red, fixed pointing up)
    p.save();
    p.translate(cx, cy);
    p.rotate(-m_heading);
    QPolygonF needle;
    needle << QPointF(0, -r+18) << QPointF(6, 8) << QPointF(0, 0) << QPointF(-6, 8);
    p.setBrush(Qt::red);
    p.setPen(Qt::NoPen);
    p.drawPolygon(needle);
    // South half (white)
    QPolygonF needleS;
    needleS << QPointF(0, r-18) << QPointF(-6, -8) << QPointF(0, 0) << QPointF(6, -8);
    p.setBrush(Qt::white);
    p.drawPolygon(needleS);
    p.restore();

    // Center dot
    p.setBrush(QColor(60,60,60));
    p.setPen(Qt::NoPen);
    p.drawEllipse(QPointF(cx, cy), 5, 5);

    // Heading value
    p.setPen(Qt::white);
    font.setPointSize(12);
    font.setBold(true);
    p.setFont(font);
    p.drawText(rect().adjusted(0, height()-30, 0, 0), Qt::AlignCenter,
               QString("%1°").arg(static_cast<int>(m_heading) % 360));

    // Title
    font.setPointSize(9);
    font.setBold(false);
    p.setFont(font);
    p.setPen(QColor(180,180,180));
    p.drawText(QRect(0, 2, width(), 20), Qt::AlignCenter, "COMPASS");
}
