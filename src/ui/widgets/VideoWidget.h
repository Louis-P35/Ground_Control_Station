#pragma once
#include <QWidget>

class QCamera;
class QMediaCaptureSession;
class QMediaDevices;
class QVideoWidget;
class QComboBox;
class QLabel;

// ---------------------------------------------------------------------------
// VideoWidget — displays a live video feed from any V4L2 / DirectShow device
// (webcam, USB video receiver, etc.).
//
// The camera list is populated from QMediaDevices::videoInputs() and refreshed
// automatically when devices are plugged or unplugged. Selecting an entry in
// the combo box starts the corresponding camera immediately.
// ---------------------------------------------------------------------------

class VideoWidget : public QWidget {
    Q_OBJECT
public:
    explicit VideoWidget(QWidget* parent = nullptr);
    ~VideoWidget() override;

private:
    void populateCameraList();
    void startCamera(const QVariant& deviceVariant);
    void stopCamera();

    QCamera*              m_camera   = nullptr;
    QMediaCaptureSession* m_session  = nullptr;
    QVideoWidget*         m_video    = nullptr;
    QMediaDevices*        m_devices  = nullptr;
    QComboBox*            m_combo    = nullptr;
    QLabel*               m_status   = nullptr;
};
