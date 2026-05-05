#include "MotorWidget.h"
#include <QPainter>
#include <QPainterPath>

MotorWidget::MotorWidget(QWidget* parent) : QWidget(parent) {
    setMinimumSize(140, 180);
}

void MotorWidget::updateData(const StatusData& d) {
    for (int i = 0; i < 4; ++i) m_motors[i] = d.motor_percent[i];
    update();
}

void MotorWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    // Title
    QFont f = p.font();
    f.setPointSize(9);
    p.setFont(f);
    p.setPen(QColor(180,180,180));
    p.drawText(QRect(0, 2, width(), 18), Qt::AlignCenter, "MOTORS");

    int titleH = 20;
    int numBarH = 18;
    int barArea  = height() - titleH - numBarH - 10;
    int barW     = (width() - 20) / 4;

    for (int i = 0; i < 4; ++i) {
        float pct   = m_motors[i] / 100.0f;
        int   bx    = 5 + i * barW;
        int   by    = titleH + 5;
        int   bh    = barArea;

        // Background
        p.setBrush(QColor(35, 35, 35));
        p.setPen(QPen(QColor(70, 70, 70), 1));
        p.drawRect(bx, by, barW - 4, bh);

        // Filled portion (from bottom up)
        int fillH = static_cast<int>(bh * pct);
        if (fillH > 0) {
            // Color interpolation: green→yellow→red
            QColor color;
            if (pct < 0.5f) {
                // green to yellow
                int r = static_cast<int>(pct * 2 * 200);
                color = QColor(r, 200, 0);
            } else {
                // yellow to red
                int g = static_cast<int>((1.0f - (pct - 0.5f) * 2) * 200);
                color = QColor(200, g, 0);
            }
            p.setBrush(color);
            p.setPen(Qt::NoPen);
            p.drawRect(bx+1, by + bh - fillH, barW - 6, fillH);
        }

        // Label M1–M4
        p.setPen(Qt::white);
        f.setPointSize(8);
        f.setBold(true);
        p.setFont(f);
        p.drawText(QRect(bx, by + bh + 2, barW - 4, 14), Qt::AlignCenter,
                   QString("M%1").arg(i+1));

        // Percentage
        f.setBold(false);
        f.setPointSize(7);
        p.setFont(f);
        p.setPen(QColor(180, 180, 180));
        p.drawText(QRect(bx, by + bh + 16, barW - 4, 12), Qt::AlignCenter,
                   QString("%1%").arg(m_motors[i]));
    }
}
