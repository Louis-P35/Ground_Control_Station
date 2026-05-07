#pragma once
#include <QList>
#include <QByteArray>
#include "backend/UdpLink.h"

// Test double that intercepts outgoing datagrams without touching the network.
// Constructed with port 0 so no socket is bound (start() is never called).
class MockUdpLink : public UdpLink {
    Q_OBJECT
public:
    explicit MockUdpLink() : UdpLink(0) {}

    // Captures every outgoing datagram instead of writing to a UDP socket.
    void sendDatagram(const QByteArray& data) override {
        sent.append(data);
    }

    // All datagrams passed to sendDatagram(), in chronological order.
    QList<QByteArray> sent;
};
