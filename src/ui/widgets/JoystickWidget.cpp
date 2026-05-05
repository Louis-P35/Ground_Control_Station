#include "JoystickWidget.h"
#include <QPainter>
#include <QFontMetrics>

static float channelToNorm(uint16_t raw) {
    // Map 1000–2000 µs to -1..+1
    return qBound(-1.0f, (static_cast<float>(raw) - 1500.0f) / 500.0f, 1.0f);
}

JoystickWidget::JoystickWidget(QWidget* parent) : QWidget(parent) {
    setMinimumSize(260, 200);
}

void JoystickWidget::updateData(const RadioData& d) {
    m_data = d;
    update();
}

void JoystickWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    // Title
    p.setPen(QColor(180,180,180));
    QFont f = p.font();
    f.setPointSize(9);
    p.setFont(f);
    p.drawText(QRect(0, 2, width(), 18), Qt::AlignCenter, "RADIO / JOYSTICK");

    // Split into left and right halves
    int half    = width() / 2;
    int stickSz = qMin(half - 20, height() - 80);
    int stickY  = 22;
    QRectF leftArea(10, stickY, stickSz, stickSz);
    QRectF rightArea(half + 10, stickY, stickSz, stickSz);

    // Ch1=Roll, Ch2=Pitch, Ch3=Throttle, Ch4=Yaw (standard AETR)
    float throttle = channelToNorm(m_data.channels[2]); // ch3
    float yaw      = channelToNorm(m_data.channels[3]); // ch4
    float pitch    = channelToNorm(m_data.channels[1]); // ch2
    float roll     = channelToNorm(m_data.channels[0]); // ch1

    drawStick(p, leftArea,  yaw, -throttle, "THR/YAW");
    drawStick(p, rightArea, roll, -pitch,   "PITCH/ROLL");

    // Channel values
    f.setPointSize(8);
    p.setFont(f);
    p.setPen(Qt::white);
    int y = stickY + stickSz + 8;
    for (int i = 0; i < 8; ++i) {
        int col = i % 4;
        int row = i / 4;
        int x   = col * (width() / 4);
        p.drawText(x, y + row * 16, width()/4, 16, Qt::AlignCenter,
                   QString("CH%1:%2").arg(i+1).arg(m_data.channels[i]));
    }

    // RSSI
    p.setPen(QColor(100, 200, 100));
    p.drawText(0, y + 34, width(), 16, Qt::AlignCenter,
               QString("RSSI: %1").arg(m_data.rssi));
}

void JoystickWidget::drawStick(QPainter& p, QRectF area, float normX, float normY, const QString& title) {
    // Background circle
    p.setPen(QPen(QColor(80,80,80), 1.5));
    p.setBrush(QColor(25,25,25));
    p.drawEllipse(area);

    // Cross-hairs
    p.setPen(QPen(QColor(60,60,60), 1));
    p.drawLine(area.center() - QPointF(area.width()/2, 0),
               area.center() + QPointF(area.width()/2, 0));
    p.drawLine(area.center() - QPointF(0, area.height()/2),
               area.center() + QPointF(0, area.height()/2));

    // Dot
    float dotR = 10;
    float dx = area.center().x() + normX * (area.width()/2  - dotR);
    float dy = area.center().y() + normY * (area.height()/2 - dotR);
    p.setBrush(QColor(80, 180, 255));
    p.setPen(Qt::NoPen);
    p.drawEllipse(QPointF(dx, dy), dotR, dotR);

    // Title
    p.setPen(QColor(150,150,150));
    QFont f = p.font();
    f.setPointSize(8);
    p.setFont(f);
    p.drawText(QRectF(area.x(), area.bottom()+2, area.width(), 14), Qt::AlignCenter, title);
}
