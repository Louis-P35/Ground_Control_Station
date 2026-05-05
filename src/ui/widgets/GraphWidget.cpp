#include "GraphWidget.h"
#include <QPainter>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QtMath>
#include <algorithm>

GraphWidget::GraphWidget(QWidget* parent) : QWidget(parent) {
    setMinimumSize(300, 180);

    m_colors = {
        QColor(255, 80,  80),   // Roll
        QColor(80,  200, 80),   // Pitch
        QColor(80,  120, 255),  // Yaw
        QColor(255, 200, 0),    // Altitude
        QColor(0,   220, 220),  // Gyro X
        QColor(220, 0,   220),  // Gyro Y
        QColor(200, 120, 0),    // Gyro Z
        QColor(180, 255, 180),  // Flow quality
    };
    m_labels = { "Roll°","Pitch°","Yaw°","Alt m","Gyr X","Gyr Y","Gyr Z","Flow Q" };

    // Checkbox row at the bottom
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(2,2,2,2);
    mainLayout->setSpacing(2);
    mainLayout->addStretch(1); // plot area takes stretch

    auto* cbRow = new QHBoxLayout();
    cbRow->setSpacing(4);
    for (int i = 0; i < NUM_CURVES; ++i) {
        m_checks[i] = new QCheckBox(m_labels[i], this);
        m_checks[i]->setChecked(true);
        m_checks[i]->setStyleSheet(
            QString("QCheckBox { color: %1; font-size: 10px; spacing: 2px; }")
                .arg(m_colors[i].name()));
        cbRow->addWidget(m_checks[i]);
        connect(m_checks[i], &QCheckBox::toggled, this, [this](bool){ update(); });
    }
    cbRow->addStretch(1);
    mainLayout->addLayout(cbRow);

    m_timerId = startTimer(10); // ~100 Hz tick
}

void GraphWidget::timerEvent(QTimerEvent*) {
    m_elapsed += 0.01f;
    sampleNow();
    update();
}

void GraphWidget::sampleNow() {
    float values[NUM_CURVES] = {
        m_roll, m_pitch, m_yaw, m_altM,
        m_gyrX, m_gyrY, m_gyrZ, m_flowQ
    };
    for (int i = 0; i < NUM_CURVES; ++i) {
        m_curves[i].push_back({m_elapsed, values[i]});
        while (!m_curves[i].empty() &&
               m_elapsed - m_curves[i].front().t > WINDOW_SECS)
            m_curves[i].pop_front();
    }
}

void GraphWidget::pushAttitude(const AttitudeData& d) {
    quatToEuler(d.qw, d.qx, d.qy, d.qz, m_roll, m_pitch, m_yaw);
    m_gyrX = d.gx; m_gyrY = d.gy; m_gyrZ = d.gz;
}
void GraphWidget::pushGps(const GpsData& d)    { m_altM  = d.altitude_m; }
void GraphWidget::pushMtf01(const Mtf01Data& d){ m_flowQ = static_cast<float>(d.quality); }

void GraphWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    // Title
    QFont f = p.font(); f.setPointSize(9); p.setFont(f);
    p.setPen(QColor(180,180,180));
    p.drawText(QRect(0, 2, width(), 16), Qt::AlignCenter, "REAL-TIME GRAPH");

    // Plot area (above checkbox row, reserve ~30px for title + 24px for checkboxes)
    int cbH = 24 + 4;
    QRectF plot(30, 20, width() - 34, height() - 20 - cbH);
    if (plot.width() < 10 || plot.height() < 10) return;

    // Background
    p.fillRect(plot, QColor(15, 15, 15));
    p.setPen(QPen(QColor(50, 50, 50), 1));
    p.drawRect(plot);

    // Compute global Y range across all visible curves
    float yMin =  1e9f, yMax = -1e9f;
    for (int i = 0; i < NUM_CURVES; ++i) {
        if (!m_checks[i]->isChecked()) continue;
        for (auto& s : m_curves[i]) {
            yMin = std::min(yMin, s.v);
            yMax = std::max(yMax, s.v);
        }
    }
    if (yMin >= yMax) { yMin -= 1; yMax += 1; }
    float yRange = yMax - yMin;
    yMin -= yRange * 0.05f;
    yMax += yRange * 0.05f;
    yRange = yMax - yMin;

    drawGrid(p, plot, yMin, yMax);

    // Draw curves
    float tEnd   = m_elapsed;
    float tStart = tEnd - WINDOW_SECS;

    auto toScreen = [&](float t, float v) -> QPointF {
        float px = plot.left() + (t - tStart) / WINDOW_SECS * plot.width();
        float py = plot.bottom() - (v - yMin) / yRange * plot.height();
        return {px, py};
    };

    for (int i = 0; i < NUM_CURVES; ++i) {
        if (!m_checks[i]->isChecked() || m_curves[i].size() < 2) continue;
        p.setPen(QPen(m_colors[i], 1.5));
        QPointF prev = toScreen(m_curves[i].front().t, m_curves[i].front().v);
        for (size_t j = 1; j < m_curves[i].size(); ++j) {
            QPointF cur = toScreen(m_curves[i][j].t, m_curves[i][j].v);
            p.drawLine(prev, cur);
            prev = cur;
        }
    }

    // Y-axis labels
    f.setPointSize(7); p.setFont(f);
    p.setPen(QColor(120,120,120));
    for (int i = 0; i <= 4; ++i) {
        float v  = yMin + yRange * i / 4.0f;
        float py = static_cast<float>(plot.bottom()) - static_cast<float>(plot.height()) * i / 4.0f;
        p.drawText(QRectF(0, py - 8, 28, 16), Qt::AlignRight | Qt::AlignVCenter,
                   QString::number(v, 'f', 1));
    }
}

void GraphWidget::drawGrid(QPainter& p, QRectF plotRect, float yMin, float yMax) const {
    p.setPen(QPen(QColor(40,40,40), 1, Qt::DashLine));
    // 5 horizontal grid lines
    for (int i = 1; i < 5; ++i) {
        float py = static_cast<float>(plotRect.bottom()) - static_cast<float>(plotRect.height()) * i / 5.0f;
        p.drawLine(QPointF(plotRect.left(), py), QPointF(plotRect.right(), py));
    }
    // 5 vertical grid lines
    for (int i = 1; i < 5; ++i) {
        float px = static_cast<float>(plotRect.left()) + static_cast<float>(plotRect.width()) * i / 5.0f;
        p.drawLine(QPointF(px, plotRect.top()), QPointF(px, plotRect.bottom()));
    }
    Q_UNUSED(yMin); Q_UNUSED(yMax);
}

void GraphWidget::quatToEuler(float qw, float qx, float qy, float qz,
                               float& roll, float& pitch, float& yaw) {
    // Standard aerospace roll/pitch/yaw from unit quaternion
    roll  = qRadiansToDegrees(std::atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy)));
    float sinp = 2*(qw*qy - qz*qx);
    pitch = qRadiansToDegrees(std::abs(sinp) >= 1 ? std::copysign(M_PI/2, sinp) : std::asin(sinp));
    yaw   = qRadiansToDegrees(std::atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz)));
}
