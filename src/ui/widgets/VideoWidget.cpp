#include "VideoWidget.h"
#include "backend/AppLogger.h"
#include <QCamera>
#include <QCameraDevice>
#include <QMediaCaptureSession>
#include <QMediaDevices>
#include <QVideoWidget>
#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>

VideoWidget::VideoWidget(QWidget* parent) : QWidget(parent) {
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(6, 6, 6, 6);
    mainLayout->setSpacing(6);

    // --- Toolbar ---
    auto* toolbar = new QHBoxLayout();
    toolbar->setSpacing(8);

    auto* camLabel = new QLabel("Camera input:", this);
    camLabel->setStyleSheet("color: #aaa; font-size: 12px;");

    m_combo = new QComboBox(this);
    m_combo->setMinimumWidth(260);
    m_combo->setStyleSheet(
        "QComboBox { background: #222; color: #ddd; border: 1px solid #444; "
        "padding: 3px 8px; border-radius: 3px; font-size: 12px; }"
        "QComboBox::drop-down { border: none; }"
        "QComboBox QAbstractItemView { background: #222; color: #ddd; "
        "selection-background-color: #2a6ac0; }");

    m_status = new QLabel(this);
    m_status->setStyleSheet("color: #666; font-size: 11px; font-style: italic;");

    // Refresh button — lets the user re-enumerate devices after granting permission
    m_refreshBtn = new QPushButton("Refresh", this);
    m_refreshBtn->setFixedHeight(26);
    m_refreshBtn->setStyleSheet(
        "QPushButton { background: #333; color: #ccc; border: 1px solid #555; "
        "padding: 2px 10px; font-size: 11px; border-radius: 3px; }"
        "QPushButton:hover { background: #444; }");

    toolbar->addWidget(camLabel);
    toolbar->addWidget(m_combo, 1);
    toolbar->addWidget(m_refreshBtn);
    toolbar->addWidget(m_status);
    mainLayout->addLayout(toolbar);

    // --- Video display ---
    m_video = new QVideoWidget(this);
    m_video->setStyleSheet("background: #000;");
    // Aspect ratio is enforced by the video renderer; fill available space
    m_video->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    mainLayout->addWidget(m_video, 1);

    // --- Capture session (persists across camera switches) ---
    m_session = new QMediaCaptureSession(this);
    m_session->setVideoOutput(m_video);

    // --- Device change notifications ---
    m_devices = new QMediaDevices(this);
    connect(m_devices, &QMediaDevices::videoInputsChanged,
            this, &VideoWidget::populateCameraList);

    // --- Combo selection → camera switch ---
    connect(m_combo, &QComboBox::currentIndexChanged, this, [this](int idx) {
        if (idx >= 0 && m_combo->itemData(idx).isValid())
            startCamera(m_combo->itemData(idx));
    });

    connect(m_refreshBtn, &QPushButton::clicked, this, &VideoWidget::populateCameraList);

    populateCameraList();
}

VideoWidget::~VideoWidget() {
    stopCamera();
}

// ---------------------------------------------------------------------------
// Device enumeration
// ---------------------------------------------------------------------------

void VideoWidget::populateCameraList() {
    // Remember current selection by device ID so we can restore it after refresh
    QString currentId = m_combo->currentData().isValid()
                        ? m_combo->currentData().value<QCameraDevice>().id()
                        : QString();

    m_combo->blockSignals(true);
    m_combo->clear();

    const QList<QCameraDevice> cameras = QMediaDevices::videoInputs();
    for (const QCameraDevice& cam : cameras)
        m_combo->addItem(cam.description(), QVariant::fromValue(cam));

    m_combo->blockSignals(false);

    if (cameras.isEmpty()) {
        AppLogger::warn("VideoWidget: no cameras detected (check OS privacy settings)");
        m_combo->addItem("No camera detected");
        m_status->setText("No camera found — check Windows Privacy Settings → Camera");
        m_status->setStyleSheet("color: #ff8844; font-size: 11px;");
        stopCamera();
        return;
    }

    AppLogger::info(QString("VideoWidget: %1 camera(s) detected").arg(cameras.size()));

    // Restore previous selection, or default to first device
    int restoreIdx = 0;
    for (int i = 0; i < cameras.size(); ++i) {
        if (cameras[i].id() == currentId) { restoreIdx = i; break; }
    }

    m_combo->setCurrentIndex(restoreIdx);

    // When items are added while signals are blocked, Qt silently sets
    // currentIndex to 0. Calling setCurrentIndex(0) again won't emit
    // currentIndexChanged, so we trigger startCamera() explicitly.
    startCamera(m_combo->itemData(restoreIdx));
}

// ---------------------------------------------------------------------------
// Camera lifecycle
// ---------------------------------------------------------------------------

void VideoWidget::startCamera(const QVariant& deviceVariant) {
    stopCamera();

    QCameraDevice device = deviceVariant.value<QCameraDevice>();
    if (device.isNull()) return;

    m_camera = new QCamera(device, this);
    m_session->setCamera(m_camera);

    // Report camera errors in the status label and log file
    connect(m_camera, &QCamera::errorOccurred, this,
            [this, device](QCamera::Error /*err*/, const QString& msg) {
        AppLogger::error(QString("VideoWidget: camera error on \"%1\": %2")
                         .arg(device.description()).arg(msg));
        m_status->setText("Error: " + msg);
        m_status->setStyleSheet("color: #ff6060; font-size: 11px;");
    });

    m_camera->start();
    AppLogger::info(QString("VideoWidget: camera started \"%1\"").arg(device.description()));
    m_status->setText(device.description());
    m_status->setStyleSheet("color: #44cc44; font-size: 11px;");
}

void VideoWidget::stopCamera() {
    if (!m_camera) return;
    AppLogger::info("VideoWidget: camera stopped");
    m_camera->stop();
    m_session->setCamera(nullptr);
    delete m_camera;
    m_camera = nullptr;
}
