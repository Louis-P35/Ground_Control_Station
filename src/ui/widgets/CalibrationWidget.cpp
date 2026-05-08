#include "CalibrationWidget.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>

// ---------------------------------------------------------------------------
// Style helpers
// ---------------------------------------------------------------------------

static const char* GROUP_STYLE =
    "QGroupBox { color: #aaa; border: 1px solid #444; border-radius: 4px; "
    "            margin-top: 12px; font-size: 13px; font-weight: bold; }"
    "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }";

static const char* DESC_STYLE =
    "color: #bbb; font-size: 12px;";

static const char* MSG_STYLE =
    "color: #888; font-size: 11px; font-style: italic;";

// Status indicator: dot color + label text per CalibStatusValue
struct StatusStyle { const char* dot; const char* text; };
static const StatusStyle STATUS_STYLES[] = {
    { "#666666", "Idle"    },  // CALIB_IDLE
    { "#ffaa00", "Running" },  // CALIB_RUNNING
    { "#44ff44", "Success" },  // CALIB_SUCCESS
    { "#ff4444", "Failed"  },  // CALIB_FAILED
};

static QPushButton* makeButton(const QString& label, const char* bg) {
    auto* btn = new QPushButton(label);
    btn->setStyleSheet(QString(
        "QPushButton { background: %1; color: white; border: 1px solid #555; "
        "              padding: 5px 14px; font-size: 12px; border-radius: 3px; }"
        "QPushButton:hover  { background: %1; filter: brightness(130%); }"
        "QPushButton:pressed { background: #222; }").arg(bg));
    btn->setFixedHeight(30);
    return btn;
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

CalibrationWidget::CalibrationWidget(QWidget* parent) : QWidget(parent) {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(12, 12, 12, 12);
    root->setSpacing(12);

    // Title banner
    auto* title = new QLabel("Sensor Calibration", this);
    title->setStyleSheet("color: white; font-size: 16px; font-weight: bold;");
    root->addWidget(title);

    // Three calibration panels side by side
    auto* row = new QHBoxLayout();
    row->setSpacing(10);
    row->addWidget(buildPanel(CALIB_ACCEL, "Accelerometer",
        "Place the drone flat on a level surface and keep it perfectly still "
        "during the entire calibration sequence.", true));
    row->addWidget(buildPanel(CALIB_MAG, "Magnetometer",
        "Slowly rotate the drone through all orientations (roll, pitch, yaw) "
        "for at least 30 seconds to map the magnetic field.", true));
    row->addWidget(buildPanel(CALIB_LEVEL, "Level / Attitude 0",
        "Place the drone on a flat surface in its normal flying orientation. "
        "This sets the zero-attitude reference used during stabilisation.", false));
    root->addLayout(row);
    root->addStretch(1);
}

// ---------------------------------------------------------------------------
// Build one calibration panel
// ---------------------------------------------------------------------------

QWidget* CalibrationWidget::buildPanel(uint8_t target,
                                       const QString& title,
                                       const QString& description,
                                       bool           hasProgress)
{
    Panel& p = m_panels[target];

    auto* box = new QGroupBox(title, this);
    box->setStyleSheet(GROUP_STYLE);
    auto* layout = new QVBoxLayout(box);
    layout->setSpacing(8);
    layout->setContentsMargins(10, 18, 10, 10);

    // Description
    auto* desc = new QLabel(description, box);
    desc->setStyleSheet(DESC_STYLE);
    desc->setWordWrap(true);
    layout->addWidget(desc);

    layout->addSpacing(4);

    // Progress bar (accelerometer + magnetometer only)
    if (hasProgress) {
        p.progress = new QProgressBar(box);
        p.progress->setRange(0, 100);
        p.progress->setValue(0);
        p.progress->setFixedHeight(16);
        p.progress->setTextVisible(true);
        p.progress->setStyleSheet(
            "QProgressBar { border: 1px solid #444; border-radius: 3px; "
            "               background: #222; color: white; font-size: 11px; }"
            "QProgressBar::chunk { background: #2a6ac0; border-radius: 2px; }");
        layout->addWidget(p.progress);
    }

    // Status row: colored dot + status text + message
    auto* statusRow = new QHBoxLayout();
    statusRow->setSpacing(6);

    p.statusDot = new QLabel("●", box);
    p.statusDot->setStyleSheet("color: #666666; font-size: 16px;");
    p.statusDot->setFixedWidth(20);
    statusRow->addWidget(p.statusDot);

    p.statusText = new QLabel("Idle", box);
    p.statusText->setStyleSheet("color: #888; font-size: 12px;");
    statusRow->addWidget(p.statusText);
    statusRow->addStretch(1);
    layout->addLayout(statusRow);

    p.msgLabel = new QLabel("", box);
    p.msgLabel->setStyleSheet(MSG_STYLE);
    p.msgLabel->setWordWrap(true);
    layout->addWidget(p.msgLabel);

    layout->addStretch(1);

    // Action buttons
    auto* btnRow = new QHBoxLayout();
    btnRow->setSpacing(6);

    if (hasProgress) {
        p.startBtn = makeButton("Start", "#1a6a1a");
        p.stopBtn  = makeButton("Stop / Save", "#6a1a1a");

        connect(p.startBtn, &QPushButton::clicked, this, [this, target] {
            emit calibCmdRequested(target, CALIB_START);
        });
        connect(p.stopBtn, &QPushButton::clicked, this, [this, target] {
            emit calibCmdRequested(target, CALIB_SAVE);
        });

        btnRow->addWidget(p.startBtn);
        btnRow->addWidget(p.stopBtn);
    } else {
        // Level calibration: single "Set Level" button
        p.startBtn = makeButton("Set Level", "#1a4a8a");
        connect(p.startBtn, &QPushButton::clicked, this, [this, target] {
            emit calibCmdRequested(target, CALIB_SAVE);
        });
        btnRow->addWidget(p.startBtn);
    }
    btnRow->addStretch(1);
    layout->addLayout(btnRow);

    return box;
}

// ---------------------------------------------------------------------------
// Live update from drone feedback
// ---------------------------------------------------------------------------

void CalibrationWidget::updateCalibStatus(uint8_t target, uint8_t status,
                                          uint8_t progress, const QString& message)
{
    if (target > 2) return;
    applyStatus(m_panels[target], status, progress, message);
}

void CalibrationWidget::applyStatus(Panel& p, uint8_t status,
                                    uint8_t progress, const QString& message)
{
    uint8_t idx = (status <= CALIB_FAILED) ? status : 0;
    const auto& s = STATUS_STYLES[idx];

    p.statusDot ->setStyleSheet(QString("color: %1; font-size: 16px;").arg(s.dot));
    p.statusText->setStyleSheet(QString("color: %1; font-size: 12px;").arg(s.dot));
    p.statusText->setText(s.text);
    p.msgLabel  ->setText(message);

    if (p.progress)
        p.progress->setValue(static_cast<int>(progress));
}
