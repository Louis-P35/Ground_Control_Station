#include "MotorWidget.h"
#include <QPainter>

MotorWidget::MotorWidget(QWidget* parent) : QWidget(parent) {
    setMinimumSize(200, 200);
}

void MotorWidget::updateData(const StatusData& d) {
    for (int i = 0; i < 8; ++i) m_motors[i] = d.motor_percent[i];
    update();
}

// ---------------------------------------------------------------------------
// X layout — 4 pairs of 2 gauges arranged at the corners of an X:
//
//   [M1 M2]      [M3 M4]
//         (center gap)
//   [M5 M6]      [M7 M8]
//
// Motor index mapping:
//   0,1 = top-left    2,3 = top-right
//   4,5 = bottom-left 6,7 = bottom-right
// ---------------------------------------------------------------------------

static void drawGauge(QPainter& p, QRect r, uint8_t pct, int motorNum) {
    float pctF = pct / 100.0f;

    // Background
    p.setBrush(QColor(35, 35, 35));
    p.setPen(QPen(QColor(70, 70, 70), 1));
    int barH = r.height() - 28; // reserve 14 for label + 14 for percent
    QRect barRect(r.left(), r.top(), r.width(), barH);
    p.drawRect(barRect);

    // Fill from bottom up
    int fillH = static_cast<int>(barH * pctF);
    if (fillH > 0) {
        QColor color;
        if (pctF < 0.5f) {
            int rv = static_cast<int>(pctF * 2 * 200);
            color = QColor(rv, 200, 0);
        } else {
            int gv = static_cast<int>((1.0f - (pctF - 0.5f) * 2) * 200);
            color = QColor(200, gv, 0);
        }
        p.setBrush(color);
        p.setPen(Qt::NoPen);
        p.drawRect(barRect.left() + 1, barRect.bottom() - fillH + 1,
                   barRect.width() - 2, fillH);
    }

    // Motor label
    QFont f = p.font();
    f.setPointSize(7);
    f.setBold(true);
    p.setFont(f);
    p.setPen(Qt::white);
    p.drawText(QRect(r.left(), r.top() + barH + 1, r.width(), 13),
               Qt::AlignCenter, QString("M%1").arg(motorNum));

    // Percentage
    f.setBold(false);
    f.setPointSize(6);
    p.setFont(f);
    p.setPen(QColor(160, 160, 160));
    p.drawText(QRect(r.left(), r.top() + barH + 14, r.width(), 13),
               Qt::AlignCenter, QString("%1%").arg(pct));
}

void MotorWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    // Title
    QFont f = p.font();
    f.setPointSize(9);
    p.setFont(f);
    p.setPen(QColor(180, 180, 180));
    p.drawText(QRect(0, 2, width(), 16), Qt::AlignCenter, "MOTORS");

    int titleH  = 20;
    int margin  = 6;
    int gapX    = width()  / 5; // center horizontal gap
    int gapY    = height() / 5; // center vertical gap
    int w       = width();
    int h       = height() - titleH;
    int pairW   = (w - gapX) / 2;      // width of one pair zone
    int pairH   = (h - gapY) / 2;      // height of one pair zone
    int gaugeW  = (pairW - 3 * margin) / 2; // width of a single gauge
    int gaugeH  = pairH - 2 * margin;

    // Top-left origin of each quadrant
    int qx[4] = { margin,           margin + pairW + gapX,
                  margin,           margin + pairW + gapX };
    int qy[4] = { titleH + margin,  titleH + margin,
                  titleH + margin + pairH + gapY,
                  titleH + margin + pairH + gapY };

    // Each quadrant holds 2 gauges side by side
    // Motors: quadrant 0 → M1,M2 | 1 → M3,M4 | 2 → M5,M6 | 3 → M7,M8
    for (int q = 0; q < 4; ++q) {
        for (int g = 0; g < 2; ++g) {
            int motorIdx = q * 2 + g;
            int x = qx[q] + g * (gaugeW + margin);
            int y = qy[q];
            drawGauge(p, QRect(x, y, gaugeW, gaugeH), m_motors[motorIdx], motorIdx + 1);
        }
    }

    // Draw subtle center X divider lines
    p.setPen(QPen(QColor(50, 50, 50), 1, Qt::DashLine));
    int cx = margin + pairW + gapX / 2;
    int cy = titleH + margin + pairH + gapY / 2;
    p.drawLine(cx, titleH, cx, height());
    p.drawLine(0, cy, width(), cy);
}
