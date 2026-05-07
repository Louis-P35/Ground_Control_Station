#include "GraphWidget.h"
#include <QPainter>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFileDialog>
#include <QTextStream>
#include <QFile>
#include <QDir>
#include <QDateTime>
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

    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(2,2,2,2);
    mainLayout->setSpacing(2);
    mainLayout->addStretch(1); // plot area takes all remaining space

    // Scrollbar — visible only when paused, allows scrubbing through session history
    m_scrollBar = new QScrollBar(Qt::Horizontal, this);
    m_scrollBar->setFixedHeight(16);
    m_scrollBar->setVisible(false);
    m_scrollBar->setStyleSheet(
        "QScrollBar:horizontal { background: #1a1a1a; height: 16px; border: 1px solid #333; }"
        "QScrollBar::handle:horizontal { background: #2a6ac0; min-width: 30px; border-radius: 3px; }"
        "QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal { width: 0; }");
    // Value unit = 0.1 s — pageStep = one full window, singleStep = 1 s
    m_scrollBar->setSingleStep(10);
    m_scrollBar->setPageStep(WINDOW_SECS * 10);
    connect(m_scrollBar, &QScrollBar::valueChanged, this, [this](int val) {
        m_viewEnd = val / 10.0f;
        update();
    });
    mainLayout->addWidget(m_scrollBar);

    // Checkbox row
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
    // While paused the display is frozen; skip the repaint to avoid useless work
    if (!m_paused)
        update();
}

void GraphWidget::sampleNow() {
    float values[NUM_CURVES] = {
        m_roll, m_pitch, m_yaw, m_altM,
        m_gyrX, m_gyrY, m_gyrZ, m_flowQ
    };
    for (int i = 0; i < NUM_CURVES; ++i) {
        Sample s{m_elapsed, values[i]};

        // Rolling display window: trim samples older than WINDOW_SECS
        m_curves[i].push_back(s);
        while (!m_curves[i].empty() &&
               m_elapsed - m_curves[i].front().t > WINDOW_SECS)
            m_curves[i].pop_front();

        // Full session history for CSV export — never trimmed
        m_history[i].push_back(s);
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

    // Title — show "PAUSED" indicator when frozen
    QFont f = p.font(); f.setPointSize(9); p.setFont(f);
    p.setPen(m_paused ? QColor(255, 160, 0) : QColor(180, 180, 180));
    QString title = m_paused ? "REAL-TIME GRAPH  [ PAUSED ]" : "REAL-TIME GRAPH";
    p.drawText(QRect(0, 2, width(), 16), Qt::AlignCenter, title);

    // Bottom reserved space: checkbox row (28px) + scrollbar when visible (18px)
    int cbH      = 24 + 4;
    int scrollH  = m_paused ? (16 + 2) : 0;
    QRectF plot(30, 20, width() - 34, height() - 20 - cbH - scrollH);
    if (plot.width() < 10 || plot.height() < 10) return;

    // Time window: live edge when playing, frozen position when paused
    float tEnd   = m_paused ? m_viewEnd : m_elapsed;
    float tStart = tEnd - WINDOW_SECS;

    // When paused we draw from the full history (m_curves may not cover the scrolled range)
    // Use binary search to locate the visible range efficiently
    auto findRange = [&](const std::deque<Sample>& src)
        -> std::pair<std::deque<Sample>::const_iterator, std::deque<Sample>::const_iterator>
    {
        auto cmp = [](const Sample& s, float t){ return s.t < t; };
        auto b = std::lower_bound(src.begin(), src.end(), tStart, cmp);
        if (b != src.begin()) --b; // one point before for line continuity
        auto e = std::upper_bound(src.begin(), src.end(), tEnd,
            [](float t, const Sample& s){ return t < s.t; });
        return {b, e};
    };

    // Compute Y range over visible samples only
    float yMin = 1e9f, yMax = -1e9f;
    for (int i = 0; i < NUM_CURVES; ++i) {
        if (!m_checks[i]->isChecked()) continue;
        const auto& src = m_paused ? m_history[i] : m_curves[i];
        auto [b, e] = findRange(src);
        for (auto it = b; it != e; ++it) {
            yMin = std::min(yMin, it->v);
            yMax = std::max(yMax, it->v);
        }
    }
    if (yMin >= yMax) { yMin -= 1; yMax += 1; }
    float yRange = yMax - yMin;
    yMin -= yRange * 0.05f;
    yMax += yRange * 0.05f;
    yRange = yMax - yMin;

    // Background + border
    p.fillRect(plot, QColor(15, 15, 15));
    p.setPen(QPen(m_paused ? QColor(80, 60, 20) : QColor(50, 50, 50), 1));
    p.drawRect(plot);

    drawGrid(p, plot, yMin, yMax);

    auto toScreen = [&](float t, float v) -> QPointF {
        float px = plot.left() + (t - tStart) / WINDOW_SECS * plot.width();
        float py = plot.bottom() - (v - yMin) / yRange * plot.height();
        return {px, py};
    };

    // Draw curves
    for (int i = 0; i < NUM_CURVES; ++i) {
        if (!m_checks[i]->isChecked()) continue;
        const auto& src = m_paused ? m_history[i] : m_curves[i];
        auto [b, e] = findRange(src);
        if (std::distance(b, e) < 2) continue;

        p.setPen(QPen(m_colors[i], 1.5));
        QPointF prev = toScreen(b->t, b->v);
        for (auto it = std::next(b); it != e; ++it) {
            QPointF cur = toScreen(it->t, it->v);
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

// ---------------------------------------------------------------------------
// Pause / Play
// ---------------------------------------------------------------------------

void GraphWidget::pause() {
    if (m_paused) return;
    m_paused  = true;
    m_viewEnd = m_elapsed;

    // Scrollbar range: min = first full window can be shown (WINDOW_SECS),
    // max = live edge at the moment of pause. Unit = 0.1 s.
    int minVal = static_cast<int>(WINDOW_SECS * 10);
    int maxVal = std::max(minVal, static_cast<int>(m_elapsed * 10));
    m_scrollBar->blockSignals(true);
    m_scrollBar->setRange(minVal, maxVal);
    m_scrollBar->setValue(maxVal); // start at live edge
    m_scrollBar->blockSignals(false);
    m_scrollBar->setVisible(true);
    update();
}

void GraphWidget::play() {
    if (!m_paused) return;
    m_paused = false;
    m_scrollBar->setVisible(false);
    update();
}

// ---------------------------------------------------------------------------
// Export
// ---------------------------------------------------------------------------

void GraphWidget::exportCsv(const PidData& pid) {
    QString defaultName = QString("telemetry_%1.csv")
        .arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));
    QString path = QFileDialog::getSaveFileName(
        this, "Export Telemetry CSV",
        QDir::homePath() + "/" + defaultName,
        "CSV Files (*.csv)");
    if (path.isEmpty()) return;

    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;
    QTextStream out(&file);

    // PID values as comment header so they are preserved alongside the data
    auto writePid = [&](const QString& name, const PidAxis& a) {
        out << QString("# %1: Kp=%2, Ki=%3, Kd=%4\n")
               .arg(name, -16)
               .arg(a.kp, 0, 'f', 6)
               .arg(a.ki, 0, 'f', 6)
               .arg(a.kd, 0, 'f', 6);
    };
    out << "# GCS Telemetry Export — " << QDateTime::currentDateTime().toString(Qt::ISODate) << "\n";
    out << "# PID Configuration\n";
    writePid("Rate Roll",       pid.rate_roll);
    writePid("Rate Pitch",      pid.rate_pitch);
    writePid("Rate Yaw",        pid.rate_yaw);
    writePid("Attitude Roll",   pid.attitude_roll);
    writePid("Attitude Pitch",  pid.attitude_pitch);
    writePid("Attitude Yaw",    pid.attitude_yaw);
    writePid("Position X",      pid.position_x);
    writePid("Position Y",      pid.position_y);
    writePid("Position Z",      pid.position_z);
    out << "#\n";

    // Column headers matching m_labels order
    out << "timestamp_s,roll_deg,pitch_deg,yaw_deg,alt_m,gyr_x_dps,gyr_y_dps,gyr_z_dps,flow_quality\n";

    // All curves are sampled simultaneously so curve[0] drives the row count
    size_t n = m_history[0].size();
    for (size_t row = 0; row < n; ++row) {
        out << QString::number(static_cast<double>(m_history[0][row].t), 'f', 3);
        for (int c = 0; c < NUM_CURVES; ++c)
            out << ',' << QString::number(static_cast<double>(m_history[c][row].v), 'f', 4);
        out << '\n';
    }
}

void GraphWidget::saveScreenshot() {
    QString defaultName = QString("graph_%1.png")
        .arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));
    QString path = QFileDialog::getSaveFileName(
        this, "Save Graph Screenshot",
        QDir::homePath() + "/" + defaultName,
        "PNG Images (*.png)");
    if (path.isEmpty()) return;

    // grab() renders the widget off-screen at its current size
    grab().save(path, "PNG");
}
