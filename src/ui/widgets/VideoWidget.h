#pragma once
#include <QWidget>

class QCamera;
class QMediaCaptureSession;
class QMediaDevices;
class QVideoWidget;
class QVideoSink;
class QComboBox;
class QLabel;
class QPushButton;

// ---------------------------------------------------------------------------
// VideoWidget — displays a live video feed from any V4L2 / DirectShow device
// (webcam, USB video receiver, etc.).
//
// The camera list is populated from QMediaDevices::videoInputs() and refreshed
// automatically when devices are plugged or unplugged. Selecting an entry in
// the combo box starts the corresponding camera immediately.
//
// Internally, frames are routed through a QVideoSink (the session's sole output)
// and pushed to QVideoWidget::videoSink(). This allows external widgets (e.g.
// MapWidget PiP) to subscribe to the same frame stream via videoSink().
// ---------------------------------------------------------------------------

class VideoWidget : public QWidget {
    Q_OBJECT
public:
    explicit VideoWidget(QWidget* parent = nullptr);
    ~VideoWidget() override;

    // Returns the primary QVideoSink fed by the capture session.
    // External consumers (MapWidget PiP) connect videoFrameChanged to their
    // own QVideoSink::setVideoFrame to receive the same frames.
    QVideoSink* videoSink() const { return m_sink; }

private:
    void populateCameraList();
    void startCamera(const QVariant& deviceVariant);
    void stopCamera();

    QCamera*              m_camera     = nullptr;
    QMediaCaptureSession* m_session    = nullptr;
    QVideoSink*           m_sink       = nullptr; // session output; feeds m_video + PiP
    QVideoWidget*         m_video      = nullptr;
    QMediaDevices*        m_devices    = nullptr;
    QComboBox*            m_combo      = nullptr;
    QLabel*               m_status     = nullptr;
    QPushButton*          m_refreshBtn = nullptr;
};
