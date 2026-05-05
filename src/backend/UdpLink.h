#pragma once
#include <QObject>
#include <QUdpSocket>
#include <QHostAddress>
#include <QTimer>
#include "PacketParser.h"

// ---------------------------------------------------------------------------
// UdpLink — lives on a dedicated QThread.
// Binds a UDP socket, receives datagrams and forwards them to PacketParser.
// Also provides sendDatagram() for outgoing packets (GCS → Drone).
// Emits connectionStateChanged when the drone link is established or lost.
// A link is considered lost after 2 s without any received packet.
// ---------------------------------------------------------------------------

class UdpLink : public QObject {
    Q_OBJECT
public:
    explicit UdpLink(quint16 listenPort = 5005, QObject* parent = nullptr);
    ~UdpLink() override;

    PacketParser* parser() const { return m_parser; }

    // Last known drone endpoint (filled from incoming datagrams)
    QHostAddress droneAddress() const { return m_droneAddress; }
    quint16      dronePort()    const { return m_dronePort;    }
    quint16      listenPort()   const { return m_listenPort;   }

    bool isConnected() const { return m_connected; }

    // UDP latency (age of last received packet in ms)
    int lastPacketAgeMs() const;

public slots:
    // Call from any thread — thread-safe via Qt::QueuedConnection
    void sendDatagram(const QByteArray& data);
    void start();

signals:
    void connectionStateChanged(bool connected);
    // Emitted from network thread — connect with Qt::QueuedConnection to UI
    void droneEndpointUpdated(QString ip, quint16 port);

private slots:
    void onReadyRead();
    void onTimeoutCheck();

private:
    QUdpSocket*  m_socket     = nullptr;
    PacketParser* m_parser    = nullptr;
    QTimer*      m_timeoutTimer = nullptr;

    quint16      m_listenPort;
    QHostAddress m_droneAddress;
    quint16      m_dronePort = 0;
    bool         m_connected = false;

    qint64       m_lastPacketMs = 0; // QDateTime::currentMSecsSinceEpoch()
};
