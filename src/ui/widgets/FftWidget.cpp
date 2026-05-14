#include "FftWidget.h"
#include <QPainter>
#include <QPen>
#include <QFont>
#include <QFontMetrics>
#include <QLabel>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFrame>
#include <QResizeEvent>
#include <cmath>
#include <algorithm>

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

FftWidget::FftWidget(QWidget* parent) : QWidget(parent)
{
    setMinimumSize(400, 300);

    // Control bar — fixed height child widget, always anchored to the top.
    // resizeEvent keeps its width in sync with the FftWidget width.
    m_ctrlBar = new QWidget(this);
    m_ctrlBar->setFixedHeight(CTRL_BAR_H);
    m_ctrlBar->setStyleSheet("background: #252525;");

    auto* hbox = new QHBoxLayout(m_ctrlBar);
    hbox->setContentsMargins(12, 6, 12, 6);
    hbox->setSpacing(6);

    auto* sensorLbl = new QLabel("Sensor:", m_ctrlBar);
    sensorLbl->setStyleSheet("color: #aaa; font-size: 11px;");

    m_accelBtn = new QPushButton("Accel", m_ctrlBar);
    m_gyroBtn  = new QPushButton("Gyro",  m_ctrlBar);

    auto* sep = new QFrame(m_ctrlBar);
    sep->setFrameShape(QFrame::VLine);
    sep->setStyleSheet("color: #444;");
    sep->setFixedWidth(1);

    auto* axisLbl = new QLabel("Axis:", m_ctrlBar);
    axisLbl->setStyleSheet("color: #aaa; font-size: 11px;");

    m_axisX = new QPushButton("X", m_ctrlBar);
    m_axisY = new QPushButton("Y", m_ctrlBar);
    m_axisZ = new QPushButton("Z", m_ctrlBar);

    for (auto* btn : {m_accelBtn, m_gyroBtn})
    {
        btn->setFixedHeight(26);
        btn->setMinimumWidth(55);
    }
    for (auto* btn : {m_axisX, m_axisY, m_axisZ})
    {
        btn->setFixedHeight(26);
        btn->setFixedWidth(36);
    }

    hbox->addWidget(sensorLbl);
    hbox->addWidget(m_accelBtn);
    hbox->addWidget(m_gyroBtn);
    hbox->addSpacing(10);
    hbox->addWidget(sep);
    hbox->addSpacing(10);
    hbox->addWidget(axisLbl);
    hbox->addWidget(m_axisX);
    hbox->addWidget(m_axisY);
    hbox->addWidget(m_axisZ);
    hbox->addStretch(1);

    connect(m_accelBtn, &QPushButton::clicked, this, [this]() { setSensor(FFT_SENSOR_ACCEL); });
    connect(m_gyroBtn,  &QPushButton::clicked, this, [this]() { setSensor(FFT_SENSOR_GYRO);  });
    connect(m_axisX,    &QPushButton::clicked, this, [this]() { setAxis(FFT_AXIS_X); });
    connect(m_axisY,    &QPushButton::clicked, this, [this]() { setAxis(FFT_AXIS_Y); });
    connect(m_axisZ,    &QPushButton::clicked, this, [this]() { setAxis(FFT_AXIS_Z); });

    updateButtonStyles();
}

// ---------------------------------------------------------------------------
// Public slot
// ---------------------------------------------------------------------------

void FftWidget::updateFft(uint8_t sensor, uint8_t axis, const FftData& data)
{
    if (sensor > 1 || axis > 2)
    {
        return;
    }
    m_data[sensor][axis] = data;

    // Only repaint when the incoming data matches what is currently displayed
    if (sensor == m_sensor && axis == m_axis)
    {
        update();
    }
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void FftWidget::setSensor(uint8_t sensor)
{
    m_sensor = sensor;
    updateButtonStyles();
    update();
}

void FftWidget::setAxis(uint8_t axis)
{
    m_axis = axis;
    updateButtonStyles();
    update();
}

void FftWidget::updateButtonStyles()
{
    static const QString kActive =
        "QPushButton { background: #1a4a8a; color: white; border: 1px solid #2a6ac0; "
        "padding: 3px 8px; font-size: 11px; border-radius: 3px; }";
    static const QString kInactive =
        "QPushButton { background: #303030; color: #999; border: 1px solid #555; "
        "padding: 3px 8px; font-size: 11px; border-radius: 3px; }"
        "QPushButton:hover { background: #404040; color: white; }";

    m_accelBtn->setStyleSheet(m_sensor == FFT_SENSOR_ACCEL ? kActive : kInactive);
    m_gyroBtn ->setStyleSheet(m_sensor == FFT_SENSOR_GYRO  ? kActive : kInactive);
    m_axisX   ->setStyleSheet(m_axis   == FFT_AXIS_X       ? kActive : kInactive);
    m_axisY   ->setStyleSheet(m_axis   == FFT_AXIS_Y       ? kActive : kInactive);
    m_axisZ   ->setStyleSheet(m_axis   == FFT_AXIS_Z       ? kActive : kInactive);
}

// ---------------------------------------------------------------------------
// resizeEvent — keep control bar stretched to full widget width
// ---------------------------------------------------------------------------

void FftWidget::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    m_ctrlBar->resize(width(), CTRL_BAR_H);
}

// ---------------------------------------------------------------------------
// paintEvent — draw the FFT plot below the control bar
// ---------------------------------------------------------------------------

void FftWidget::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    // Full widget background
    p.fillRect(rect(), QColor(20, 20, 20));

    // Plot margins (inside the area below the control bar)
    const int marginL = 68;
    const int marginR = 20;
    const int marginT = 12;
    const int marginB = 46;

    QRect plotRect(
        marginL,
        CTRL_BAR_H + marginT,
        width()  - marginL - marginR,
        height() - CTRL_BAR_H - marginT - marginB
    );

    if (plotRect.width() < 20 || plotRect.height() < 20)
    {
        return;
    }

    // ── Data check ────────────────────────────────────────────────────────────

    const FftData& d = m_data[m_sensor][m_axis];

    if (!d.valid || d.bin_count == 0 || d.freq_resolution_hz <= 0.0f)
    {
        p.setPen(QColor(100, 100, 100));
        p.setFont(QFont("Arial", 13));
        p.drawText(plotRect, Qt::AlignCenter, "No FFT data received");
        return;
    }

    // ── Convert magnitudes to dB (20·log10, clamped to avoid log(0)) ─────────

    auto toDb = [](float mag) -> float
    {
        return 20.0f * std::log10(std::max(mag, 1e-9f));
    };

    // Y range: peak of the raw spectrum defines the top; 70 dB of dynamic range
    float peakDb = -300.0f;
    for (int i = 1; i < d.bin_count; ++i) // skip DC bin (i=0)
    {
        peakDb = std::max(peakDb, toDb(d.raw[i]));
    }
    if (peakDb < -200.0f)
    {
        p.setPen(QColor(100, 100, 100));
        p.setFont(QFont("Arial", 13));
        p.drawText(plotRect, Qt::AlignCenter, "No FFT data received");
        return;
    }

    float yMax = peakDb + 5.0f;
    float yMin = yMax - 70.0f;

    // X range: 0 Hz to Nyquist (last bin × freq resolution)
    float xMax = static_cast<float>(d.bin_count - 1) * d.freq_resolution_hz;

    // ── Coordinate mapping ────────────────────────────────────────────────────

    auto mapX = [&](float hz) -> float
    {
        return static_cast<float>(plotRect.left())
             + (hz / xMax) * static_cast<float>(plotRect.width());
    };

    auto mapY = [&](float db) -> float
    {
        float norm = (db - yMin) / (yMax - yMin);
        return static_cast<float>(plotRect.bottom())
             - norm * static_cast<float>(plotRect.height());
    };

    // ── Grid background ───────────────────────────────────────────────────────

    p.fillRect(plotRect, QColor(15, 15, 15));

    // Horizontal grid — one line every 10 dB
    int dbMin = static_cast<int>(std::ceil (yMin / 10.0f)) * 10;
    int dbMax = static_cast<int>(std::floor(yMax / 10.0f)) * 10;

    p.setFont(QFont("Arial", 9));

    for (int db = dbMin; db <= dbMax; db += 10)
    {
        float y = mapY(static_cast<float>(db));
        p.setPen(QPen(QColor(45, 45, 45), 1, Qt::DashLine));
        p.drawLine(QPointF(plotRect.left(), y), QPointF(plotRect.right(), y));

        p.setPen(QColor(150, 150, 150));
        p.drawText(QRectF(2, y - 8, marginL - 6, 16),
                   Qt::AlignRight | Qt::AlignVCenter,
                   QString("%1 dB").arg(db));
    }

    // Vertical grid — auto-spaced at round Hz values
    float hzStep = 50.0f;
    if (xMax > 400.0f)  { hzStep = 100.0f; }
    if (xMax > 1000.0f) { hzStep = 200.0f; }
    if (xMax > 2000.0f) { hzStep = 500.0f; }

    for (float hz = 0.0f; hz <= xMax + 0.5f * hzStep; hz += hzStep)
    {
        float x = mapX(hz);
        if (x < plotRect.left() || x > plotRect.right())
        {
            continue;
        }
        p.setPen(QPen(QColor(45, 45, 45), 1, Qt::DashLine));
        p.drawLine(QPointF(x, plotRect.top()), QPointF(x, plotRect.bottom()));

        p.setPen(QColor(150, 150, 150));
        p.drawText(QRectF(x - 28, plotRect.bottom() + 6, 56, 16),
                   Qt::AlignCenter,
                   QString("%1 Hz").arg(static_cast<int>(hz)));
    }

    // Plot border
    p.setPen(QPen(QColor(70, 70, 70), 1));
    p.drawRect(plotRect);

    // ── Y-axis label (rotated) ────────────────────────────────────────────────

    p.save();
    p.translate(10, plotRect.center().y());
    p.rotate(-90);
    p.setFont(QFont("Arial", 10));
    p.setPen(QColor(180, 180, 180));
    p.drawText(QRectF(-40, -10, 80, 20), Qt::AlignCenter, "Magnitude (dB)");
    p.restore();

    // X-axis title
    p.setPen(QColor(180, 180, 180));
    p.setFont(QFont("Arial", 10));
    p.drawText(QRectF(plotRect.left(), plotRect.bottom() + 28,
                      plotRect.width(), 16),
               Qt::AlignCenter, "Frequency (Hz)");

    // ── Curves ───────────────────────────────────────────────────────────────
    // Clip drawing to the plot area so curves don't bleed into labels
    p.setClipRect(plotRect);

    struct CurveDef
    {
        const float* data;
        QColor       color;
        const char*  label;
    };

    const CurveDef curves[3] = {
        { d.raw,   QColor( 70, 130, 255), "Raw"        },
        { d.notch, QColor(255, 160,   0), "Notch"      },
        { d.full,  QColor( 50, 210,  50), "Notch+Pass" },
    };

    for (const auto& c : curves)
    {
        p.setPen(QPen(c.color, 1.4f));

        QPolygonF poly;
        poly.reserve(d.bin_count);

        for (int i = 1; i < d.bin_count; ++i) // skip DC bin
        {
            float hz = static_cast<float>(i) * d.freq_resolution_hz;
            float db = toDb(c.data[i]);
            // Clamp to plot range so out-of-range values are drawn on the edge
            db = std::max(db, yMin);
            db = std::min(db, yMax);
            poly << QPointF(mapX(hz), mapY(db));
        }
        p.drawPolyline(poly);
    }

    p.setClipping(false);

    // ── Legend (top-right of plot area) ──────────────────────────────────────

    int legX = plotRect.right() - 130;
    int legY = plotRect.top() + 12;

    // Translucent background
    p.fillRect(legX - 6, legY - 6, 130, 3 * 20 + 4, QColor(0, 0, 0, 140));

    p.setFont(QFont("Arial", 9));
    for (const auto& c : curves)
    {
        p.setPen(QPen(c.color, 2));
        p.drawLine(legX, legY + 6, legX + 18, legY + 6);
        p.setPen(Qt::white);
        p.drawText(legX + 24, legY + 11, c.label);
        legY += 20;
    }

    // ── Sensor/axis label (top-left of plot area) ─────────────────────────────

    static const char* kSensorNames[2] = { "Accelerometer", "Gyroscope" };
    static const char* kAxisNames[3]   = { "X", "Y", "Z" };

    QString label = QString("%1 – Axis %2")
                        .arg(kSensorNames[m_sensor])
                        .arg(kAxisNames[m_axis]);

    p.setFont(QFont("Arial", 10, QFont::Bold));
    p.setPen(QColor(200, 200, 200));
    p.drawText(QRectF(plotRect.left() + 6, plotRect.top() + 6, 300, 18),
               Qt::AlignLeft | Qt::AlignVCenter, label);
}
