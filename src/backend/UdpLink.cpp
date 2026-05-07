#include "UdpLink.h"
#include "AppLogger.h"
#include <QDateTime>

static constexpr int CONNECTION_TIMEOUT_MS = 2000;
static constexpr int TIMEOUT_CHECK_INTERVAL_MS = 500;

UdpLink::UdpLink(quint16 listenPort, QObject* parent)
    : QObject(parent)
    , m_listenPort(listenPort)
{
    m_parser = new PacketParser(this);
}

UdpLink::~UdpLink() = default;

void UdpLink::start() {
    m_socket = new QUdpSocket(this);
    m_socket->bind(QHostAddress::AnyIPv4, m_listenPort, QUdpSocket::ShareAddress);
    AppLogger::info(QString("UdpLink: listening on UDP port %1").arg(m_listenPort));
    connect(m_socket, &QUdpSocket::readyRead, this, &UdpLink::onReadyRead);

    m_timeoutTimer = new QTimer(this);
    m_timeoutTimer->setInterval(TIMEOUT_CHECK_INTERVAL_MS);
    connect(m_timeoutTimer, &QTimer::timeout, this, &UdpLink::onTimeoutCheck);
    m_timeoutTimer->start();
}

void UdpLink::onReadyRead() {
    while (m_socket->hasPendingDatagrams()) {
        QByteArray datagram;
        QHostAddress sender;
        quint16 senderPort;
        datagram.resize(static_cast<int>(m_socket->pendingDatagramSize()));
        m_socket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

        // Update drone endpoint if changed
        if (sender != m_droneAddress || senderPort != m_dronePort) {
            m_droneAddress = sender;
            m_dronePort    = senderPort;
            AppLogger::info(QString("UdpLink: drone endpoint updated to %1:%2")
                            .arg(sender.toString()).arg(senderPort));
            emit droneEndpointUpdated(sender.toString(), senderPort);
        }

        m_lastPacketMs = QDateTime::currentMSecsSinceEpoch();

        if (!m_connected) {
            m_connected = true;
            AppLogger::info(QString("UdpLink: connected to drone at %1:%2")
                            .arg(m_droneAddress.toString()).arg(m_dronePort));
            emit connectionStateChanged(true);
        }

        m_parser->parse(datagram);
    }
}

void UdpLink::onTimeoutCheck() {
    if (!m_connected) return;
    qint64 now = QDateTime::currentMSecsSinceEpoch();
    if (now - m_lastPacketMs > CONNECTION_TIMEOUT_MS) {
        m_connected = false;
        AppLogger::warn("UdpLink: connection lost (packet timeout)");
        emit connectionStateChanged(false);
    }
}

void UdpLink::sendDatagram(const QByteArray& data) {
    if (m_socket && m_dronePort != 0)
        m_socket->writeDatagram(data, m_droneAddress, m_dronePort);
}

int UdpLink::lastPacketAgeMs() const {
    if (m_lastPacketMs == 0) return -1;
    return static_cast<int>(QDateTime::currentMSecsSinceEpoch() - m_lastPacketMs);
}
