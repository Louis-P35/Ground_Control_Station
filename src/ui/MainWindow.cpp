#include "MainWindow.h"
#include <QGridLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QWidget>
#include <QStatusBar>
#include <QApplication>
#include <QtMath>
#include "widgets/DroneWidget3D.h"
#include "widgets/TrackingWidget3D.h"
#include "widgets/CompassWidget.h"
#include "widgets/JoystickWidget.h"
#include "widgets/Mtf01Widget.h"
#include "widgets/GpsWidget.h"
#include "widgets/MotorWidget.h"
#include "widgets/StatusWidget.h"
#include "widgets/StatusWidget.h"
#include "widgets/PidConfigWidget.h"
#include "widgets/BarometerWidget.h"
#include "widgets/GraphWidget.h"
#include "widgets/TerminalWidget.h"
#include "widgets/MapWidget.h"
#include "widgets/VideoWidget.h"
#include "widgets/CalibrationWidget.h"
#include "widgets/FftWidget.h"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    setWindowTitle("Ground Control Station");
    setMinimumSize(1400, 900);

    // Apply dark theme
    qApp->setStyle("Fusion");
    QPalette dark;
    dark.setColor(QPalette::Window,          QColor(30, 30, 30));
    dark.setColor(QPalette::WindowText,      Qt::white);
    dark.setColor(QPalette::Base,            QColor(20, 20, 20));
    dark.setColor(QPalette::AlternateBase,   QColor(40, 40, 40));
    dark.setColor(QPalette::Text,            Qt::white);
    dark.setColor(QPalette::Button,          QColor(50, 50, 50));
    dark.setColor(QPalette::ButtonText,      Qt::white);
    dark.setColor(QPalette::Highlight,       QColor(42, 130, 218));
    dark.setColor(QPalette::HighlightedText, Qt::black);
    qApp->setPalette(dark);

    setupUi();
    setupStatusBar();

    // Network — lives on its own thread
    m_netThread = new QThread(this);
    m_udpLink   = new UdpLink(5005);
    m_udpLink->moveToThread(m_netThread);
    connect(m_netThread, &QThread::started, m_udpLink, &UdpLink::start);

    m_cmdSender = new CommandSender(m_udpLink);
    m_cmdSender->moveToThread(m_netThread);

    connectSignals();

    m_netThread->start();

    m_statusTimer = new QTimer(this);
    m_statusTimer->setInterval(500);
    connect(m_statusTimer, &QTimer::timeout, this, &MainWindow::onStatusBarTick);
    m_statusTimer->start();
}

MainWindow::~MainWindow() {
    m_netThread->quit();
    m_netThread->wait();
    delete m_udpLink;
    delete m_cmdSender;
}

// ---------------------------------------------------------------------------
// UI layout:
//
//  Tab 0 "Dashboard":
//    Col:   0              1              2              3
//    Row 0: [3D (r0-1, c0-1)             ] [joystick      ] [motors (r0-1)]
//    Row 1: [3D                          ] [compass       ] [motors       ]
//    Row 2: [terminal x2                   ] [baro|gps|status|mtf01         ]
//
//  Tab 1 "3D Tracking": Third-person follow view (drone + trail in NWU scene)
//  Tab 2 "Graph":       Full-window graph widget
//  Tab 3 "Map":         Full-window map widget
//  Tab 4 "Video":       Full-window video feed
//  Tab 5 "Settings":    PID configuration
//  Tab 6 "Calibration": Sensor calibration (accel, mag, level)
// ---------------------------------------------------------------------------
void MainWindow::setupUi() {
    m_drone3d  = new DroneWidget3D(this);
    m_tracking = new TrackingWidget3D(this);
    m_compass  = new CompassWidget(this);
    m_joystick = new JoystickWidget(this);
    m_mtf01    = new Mtf01Widget(this);
    m_gps      = new GpsWidget(this);
    m_motor    = new MotorWidget(this);
    m_status   = new StatusWidget(this);
    m_pid        = new PidConfigWidget(this);
    m_barometer  = new BarometerWidget(this);
    m_graph       = new GraphWidget(this);
    m_terminal    = new TerminalWidget(this);
    m_map         = new MapWidget(this);
    m_video       = new VideoWidget(this);
    m_calibration = new CalibrationWidget(this);
    m_fft         = new FftWidget(this);

    // --- Tab 0: Dashboard ---
    auto* dashTab = new QWidget(this);
    auto* grid = new QGridLayout(dashTab);
    grid->setSpacing(6);
    grid->setContentsMargins(6, 6, 6, 6);

    // Rows 0-1, col 0: 3D view — same width as original, double height.
    // Joystick sits above compass in col 2; col 1 is intentionally empty.
    grid->addWidget(m_drone3d,  0, 0, 2, 2);
    grid->addWidget(m_joystick, 0, 2);        // row 0, above compass
    grid->addWidget(m_compass,  1, 2);        // row 1, below joystick
    grid->addWidget(m_motor,    0, 3, 2, 1);

    // Row 2: terminal (left half) + info bar (right half).
    // GPS is now in the info bar (swapped with compass).
    auto* infoBar   = new QWidget(dashTab);
    auto* infoHBox  = new QHBoxLayout(infoBar);
    infoHBox->setContentsMargins(0, 0, 0, 0);
    infoHBox->setSpacing(4);
    infoHBox->addWidget(m_barometer);
    infoHBox->addWidget(m_gps);     // swapped: GPS replaces compass here
    infoHBox->addWidget(m_status);
    infoHBox->addWidget(m_mtf01);

    grid->addWidget(m_terminal, 2, 0, 1, 2);
    grid->addWidget(infoBar,    2, 2, 1, 2);

    grid->setColumnStretch(0, 2);
    grid->setColumnStretch(1, 2);
    grid->setColumnStretch(2, 2);
    grid->setColumnStretch(3, 2);

    // Rows 0-1 hold the main widgets; row 2 (terminal) gets more vertical space.
    grid->setRowStretch(0, 2);
    grid->setRowStretch(1, 2);
    grid->setRowStretch(2, 3);

    // --- Tab 1: Graph ---
    auto* graphTab = new QWidget(this);
    auto* graphLayout = new QVBoxLayout(graphTab);
    graphLayout->setContentsMargins(6, 6, 6, 6);
    graphLayout->setSpacing(4);

    // Toolbar with playback controls and export actions
    auto* btnBar  = new QHBoxLayout();
    auto* pauseBtn = new QPushButton("Pause",       graphTab);
    auto* playBtn  = new QPushButton("Play",        graphTab);
    auto* csvBtn   = new QPushButton("Export CSV",  graphTab);
    auto* ssBtn    = new QPushButton("Screenshot",  graphTab);

    QString btnStyle = "QPushButton { background: #1a4a8a; color: white; border: 1px solid #2a6ac0; "
                       "padding: 4px 12px; font-size: 12px; border-radius: 3px; }"
                       "QPushButton:hover { background: #2a6ac0; }";
    QString pauseStyle = "QPushButton { background: #7a4a00; color: white; border: 1px solid #c07000; "
                         "padding: 4px 12px; font-size: 12px; border-radius: 3px; }"
                         "QPushButton:hover { background: #c07000; }";
    pauseBtn->setStyleSheet(pauseStyle);
    playBtn ->setStyleSheet(btnStyle);
    csvBtn  ->setStyleSheet(btnStyle);
    ssBtn   ->setStyleSheet(btnStyle);
    for (auto* b : {pauseBtn, playBtn, csvBtn, ssBtn}) {
        b->setFixedHeight(28);
        btnBar->addWidget(b);
    }
    btnBar->addStretch(1);
    graphLayout->addLayout(btnBar);
    graphLayout->addWidget(m_graph);

    connect(pauseBtn, &QPushButton::clicked, m_graph, &GraphWidget::pause);
    connect(playBtn,  &QPushButton::clicked, m_graph, &GraphWidget::play);
    // Export CSV: pass current PID state to the graph so it can write the header
    connect(csvBtn, &QPushButton::clicked, this, [this]() {
        m_graph->exportCsv(m_state.pid());
    });
    connect(ssBtn, &QPushButton::clicked, m_graph, &GraphWidget::saveScreenshot);

    // Wire the Map PiP to the VideoWidget's QVideoSink.
    // Both VideoWidget's QVideoWidget and the PiP subscribe to videoFrameChanged
    // so they receive identical frames without opening the camera a second time.
    m_map->setPipSink(m_video->videoSink());

    // --- Tab 4: Settings — PID configuration ---
    auto* settingsTab = new QWidget(this);
    auto* settingsLayout = new QVBoxLayout(settingsTab);
    settingsLayout->setContentsMargins(12, 12, 12, 12);
    settingsLayout->setSpacing(8);
    settingsLayout->addWidget(m_pid);
    settingsLayout->addStretch(1);

    // --- Tab widget ---
    m_tabs = new QTabWidget(this);
    m_tabs->setDocumentMode(false);
    m_tabs->addTab(dashTab,      "Dashboard");

    // --- Tab 1: 3D Tracking — third-person follow view of the drone ---
    m_tabs->addTab(m_tracking,   "3D Tracking");

    m_tabs->addTab(graphTab,     "Graph");

    // --- Tab 2: Map — MapWidget manages its own overlay buttons internally ---
    m_tabs->addTab(m_map,        "Map");

    // --- Tab 3: Video — live feed from a USB camera or video receiver ---
    m_tabs->addTab(m_video,      "Video");

    // --- Tab 4: Settings — PID tuning parameters ---
    m_tabs->addTab(settingsTab,  "Settings");

    // --- Tab 5: Calibration — on-ground sensor calibration ---
    m_tabs->addTab(m_calibration, "Calibration");

    // --- Tab 6: FFT — frequency spectrum of gyroscope/accelerometer signals ---
    m_tabs->addTab(m_fft, "FFT");

    setCentralWidget(m_tabs);
}

void MainWindow::setupStatusBar() {
    auto* sb = statusBar();
    sb->setStyleSheet("QStatusBar { background: #1a1a1a; color: white; }");

    m_statusConn    = new QLabel("  Disconnected  ", this);
    m_statusLatency = new QLabel("  Latency: -- ms  ", this);
    m_statusRssi    = new QLabel("  RSSI: --  ", this);
    m_statusLoss    = new QLabel("  Loss: -- %  ", this);
    m_statusPort    = new QLabel("  Port: 5005  ", this);
    m_statusDroneIp = new QLabel("  Drone: --  ", this);

    m_statusConn->setStyleSheet("color: #ff4444; font-weight: bold;");

    sb->addWidget(m_statusConn);
    sb->addWidget(m_statusLatency);
    sb->addWidget(m_statusRssi);
    sb->addWidget(m_statusLoss);
    sb->addPermanentWidget(m_statusPort);
    sb->addPermanentWidget(m_statusDroneIp);
}

void MainWindow::connectSignals() {
    auto* parser = m_udpLink->parser();

    // Qt::QueuedConnection because parser lives on network thread
    connect(m_udpLink, &UdpLink::connectionStateChanged,
            this,      &MainWindow::onConnectionChanged,    Qt::QueuedConnection);
    connect(m_udpLink, &UdpLink::droneEndpointUpdated,
            this,      &MainWindow::onDroneEndpointUpdated, Qt::QueuedConnection);
    connect(parser, &PacketParser::attitudeReceived,
            this,   &MainWindow::onAttitudeReceived,        Qt::QueuedConnection);
    connect(parser, &PacketParser::gpsReceived,
            this,   &MainWindow::onGpsReceived,             Qt::QueuedConnection);
    connect(parser, &PacketParser::mtf01Received,
            this,   &MainWindow::onMtf01Received,           Qt::QueuedConnection);
    connect(parser, &PacketParser::radioReceived,
            this,   &MainWindow::onRadioReceived,           Qt::QueuedConnection);
    connect(parser, &PacketParser::statusReceived,
            this,   &MainWindow::onStatusReceived,          Qt::QueuedConnection);
    connect(parser, &PacketParser::positionReceived,
            this,   &MainWindow::onPositionReceived,        Qt::QueuedConnection);
    connect(parser, &PacketParser::magReceived,
            this,   &MainWindow::onMagReceived,             Qt::QueuedConnection);
    connect(parser, &PacketParser::pidReceived,
            this,   &MainWindow::onPidReceived,             Qt::QueuedConnection);
    connect(parser, &PacketParser::baroReceived,
            this,   &MainWindow::onBaroReceived,            Qt::QueuedConnection);
    connect(parser, &PacketParser::calibStatusReceived,
            this,   &MainWindow::onCalibStatusReceived,     Qt::QueuedConnection);
    connect(parser, &PacketParser::fftReceived,
            this,   &MainWindow::onFftReceived,             Qt::QueuedConnection);
    connect(parser, &PacketParser::logReceived,
            this,   &MainWindow::onLogReceived,             Qt::QueuedConnection);

    // Calibration widget → command sender (invoked on network thread)
    connect(m_calibration, &CalibrationWidget::calibCmdRequested,
            this, [this](uint8_t target, uint8_t action) {
        QMetaObject::invokeMethod(m_cmdSender, [this, target, action] {
            m_cmdSender->sendCalibCmd(target, action);
        }, Qt::QueuedConnection);
    });

    // PID config widget → command sender (invoked on network thread)
    connect(m_pid, &PidConfigWidget::sendPidRequested,
            this, [this](PidAxisId axis, float kp, float ki, float kd) {
        QMetaObject::invokeMethod(m_cmdSender, [this, axis, kp, ki, kd] {
            m_cmdSender->sendSetPid(axis, kp, ki, kd);
        }, Qt::QueuedConnection);
    });
}

// ---------------------------------------------------------------------------
// Slots
// ---------------------------------------------------------------------------
void MainWindow::onConnectionChanged(bool connected) {
    if (connected) {
        m_statusConn->setText("  Connected  ");
        m_statusConn->setStyleSheet("color: #44ff44; font-weight: bold;");
    } else {
        m_statusConn->setText("  Disconnected  ");
        m_statusConn->setStyleSheet("color: #ff4444; font-weight: bold;");
    }
}

void MainWindow::onDroneEndpointUpdated(QString ip, quint16 /*port*/) {
    m_statusDroneIp->setText(QString("  Drone: %1  ").arg(ip));
}

void MainWindow::onAttitudeReceived(AttitudeData d) {
    m_state.updateAttitude(d);
    m_drone3d->updateAttitude(d);
    m_tracking->updateAttitude(d);
    m_graph->pushAttitude(d);
}

void MainWindow::onGpsReceived(GpsData d) {
    m_state.updateGps(d);
    m_gps->updateData(d);
    m_graph->pushGps(d);
    m_map->updatePosition(d);
    // Note: the compass is driven by the magnetometer (onMagReceived), not the
    // GPS course-over-ground, which is only meaningful while the drone is moving.
}

void MainWindow::onMtf01Received(Mtf01Data d) {
    m_state.updateMtf01(d);
    m_mtf01->updateData(d);
    m_graph->pushMtf01(d);
}

void MainWindow::onRadioReceived(RadioData d) {
    m_state.updateRadio(d);
    m_joystick->updateData(d);
}

void MainWindow::onStatusReceived(StatusData d) {
    m_state.updateStatus(d);
    m_motor->updateData(d);
    m_status->updateData(d);
    m_statusRssi->setText(QString("  RSSI: %1  ").arg(d.wifi_rssi));
}

void MainWindow::onPositionReceived(PositionData d) {
    m_state.updatePosition(d);
    m_tracking->updatePosition(d);
}

void MainWindow::onMagReceived(MagData d) {
    m_state.updateMag(d);

    // Derive a compass heading from the horizontal magnetometer components.
    // The drone world frame is NWU (X=North, Y=West): a compass heading is the
    // clockwise angle from North toward East, i.e. atan2(East, North) with
    // East = -West = -y. Normalised to [0, 360).
    float heading = qRadiansToDegrees(std::atan2(-static_cast<float>(d.y),
                                                  static_cast<float>(d.x)));
    if (heading < 0.0f)
        heading += 360.0f;
    m_compass->setHeading(heading);
}

void MainWindow::onPidReceived(PidData d) {
    m_state.updatePid(d);
    m_pid->updatePid(d);
}

void MainWindow::onBaroReceived(BaroData d) {
    m_state.updateBaro(d);
    m_barometer->updateData(d);
}

void MainWindow::onCalibStatusReceived(uint8_t target, uint8_t status,
                                       uint8_t progress, QString message) {
    CalibStatusData csd;
    csd.status   = status;
    csd.progress = progress;
    csd.message  = message.toStdString();
    m_state.updateCalibStatus(target, csd);
    m_calibration->updateCalibStatus(target, status, progress, message);
}

void MainWindow::onFftReceived(uint8_t sensor, uint8_t axis, FftData d) {
    m_state.updateFft(sensor, axis, d);
    m_fft->updateFft(sensor, axis, d);
}

void MainWindow::onLogReceived(uint8_t level, QString text) {
    m_terminal->appendMessage(level, text);
}

void MainWindow::onStatusBarTick() {
    int ageMs = m_udpLink->lastPacketAgeMs();
    if (ageMs >= 0)
        m_statusLatency->setText(QString("  Latency: %1 ms  ").arg(ageMs));
    else
        m_statusLatency->setText("  Latency: -- ms  ");

    float loss = m_udpLink->parser()->packetLoss(PKT_ATTITUDE);
    m_statusLoss->setText(QString("  Loss: %1 %  ").arg(loss, 0, 'f', 1));
}
