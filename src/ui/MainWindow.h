#pragma once
#include <QMainWindow>
#include <QThread>
#include <QLabel>
#include <QTimer>
#include <QTabWidget>
#include "backend/UdpLink.h"
#include "backend/TelemetryState.h"
#include "backend/CommandSender.h"

class DroneWidget3D;
class CompassWidget;
class JoystickWidget;
class Mtf01Widget;
class GpsWidget;
class MotorWidget;
class StatusWidget;
class PidConfigWidget;
class GraphWidget;
class TerminalWidget;
class MapWidget;

// ---------------------------------------------------------------------------
// MainWindow — single application window.
// Owns the network thread and all UI widgets.
// ---------------------------------------------------------------------------

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

private slots:
    void onConnectionChanged(bool connected);
    void onDroneEndpointUpdated(QString ip, quint16 port);
    void onAttitudeReceived(AttitudeData d);
    void onGpsReceived(GpsData d);
    void onMtf01Received(Mtf01Data d);
    void onRadioReceived(RadioData d);
    void onStatusReceived(StatusData d);
    void onPidReceived(PidData d);
    void onLogReceived(uint8_t level, QString text);
    void onStatusBarTick();

private:
    void setupUi();
    void setupStatusBar();
    void connectSignals();

    // Network
    QThread*       m_netThread = nullptr;
    UdpLink*       m_udpLink   = nullptr;
    TelemetryState m_state;
    CommandSender* m_cmdSender = nullptr;

    // Tab container
    QTabWidget*      m_tabs     = nullptr;

    // Widgets
    DroneWidget3D*   m_drone3d  = nullptr;
    CompassWidget*   m_compass  = nullptr;
    JoystickWidget*  m_joystick = nullptr;
    Mtf01Widget*     m_mtf01    = nullptr;
    GpsWidget*       m_gps      = nullptr;
    MotorWidget*     m_motor    = nullptr;
    StatusWidget*    m_status   = nullptr;
    PidConfigWidget* m_pid      = nullptr;
    GraphWidget*     m_graph    = nullptr;
    TerminalWidget*  m_terminal = nullptr;
    MapWidget*       m_map      = nullptr;

    // Status bar labels
    QLabel* m_statusConn    = nullptr;
    QLabel* m_statusLatency = nullptr;
    QLabel* m_statusRssi    = nullptr;
    QLabel* m_statusLoss    = nullptr;
    QLabel* m_statusPort    = nullptr;
    QLabel* m_statusDroneIp = nullptr;

    QTimer* m_statusTimer = nullptr;
};
