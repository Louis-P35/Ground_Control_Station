#include "VideoWidget.h"
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
#include <QGroupBox>

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

    toolbar->addWidget(camLabel);
    toolbar->addWidget(m_combo, 1);
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
        if (idx >= 0)
            startCamera(m_combo->itemData(idx));
    });

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
    QString currentId = m_combo->currentData().value<QCameraDevice>().id();

    m_combo->blockSignals(true);
    m_combo->clear();

    const QList<QCameraDevice> cameras = QMediaDevices::videoInputs();
    for (const QCameraDevice& cam : cameras)
        m_combo->addItem(cam.description(), QVariant::fromValue(cam));

    m_combo->blockSignals(false);

    if (cameras.isEmpty()) {
        m_combo->addItem("No camera detected");
        m_status->setText("Plug in a camera or USB video receiver");
        stopCamera();
        return;
    }

    // Restore previous selection, or default to first device
    int restoreIdx = 0;
    for (int i = 0; i < cameras.size(); ++i) {
        if (cameras[i].id() == currentId) { restoreIdx = i; break; }
    }

    // Setting currentIndex emits currentIndexChanged → startCamera
    m_combo->setCurrentIndex(restoreIdx);
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

    // Report camera errors in the status label
    connect(m_camera, &QCamera::errorOccurred, this,
            [this](QCamera::Error /*err*/, const QString& msg) {
        m_status->setText("Error: " + msg);
        m_status->setStyleSheet("color: #ff6060; font-size: 11px;");
    });

    m_camera->start();
    m_status->setText(device.description());
    m_status->setStyleSheet("color: #44cc44; font-size: 11px;");
}

void VideoWidget::stopCamera() {
    if (!m_camera) return;
    m_camera->stop();
    m_session->setCamera(nullptr);
    delete m_camera;
    m_camera = nullptr;
}
