#pragma once
#include <QObject>
#include <QTimer>
#include <QMap>
#include "Protocol.h" // shared with ESP32 firmware — lives in common/
#include "UdpLink.h"

// ---------------------------------------------------------------------------
// CommandSender — builds outgoing packets and handles the ACK retry loop.
//
// For PKT_SET_PID: retries up to 3 times with a 200 ms timeout if no ACK.
// Listens to UdpLink::parser()'s ackReceived signal to clear pending retries.
// ---------------------------------------------------------------------------

class CommandSender : public QObject {
    Q_OBJECT
public:
    explicit CommandSender(UdpLink* link, QObject* parent = nullptr);

    // Send a single PID axis update. Queues the packet for ACK-backed delivery.
    void sendSetPid(PidAxisId axis, float kp, float ki, float kd);

    // Send a calibration command (start/stop/save). ACK-backed.
    void sendCalibCmd(uint8_t target, uint8_t action);

private slots:
    void onAckReceived(uint8_t ackType, uint16_t ackSeq, uint8_t success);
    void onRetryTimer();

private:
    struct PendingCmd {
        QByteArray data;
        int        retriesLeft = 3;
        uint16_t   seq;
    };

    QByteArray buildSetPid  (PidAxisId axis, float kp, float ki, float kd, uint16_t seq);
    QByteArray buildCalibCmd(uint8_t target, uint8_t action, uint16_t seq);
    static uint16_t crc16(const uint8_t* data, int len);

    UdpLink*  m_link;
    QTimer*   m_retryTimer;
    uint16_t  m_seqCounter = 0;

    // seq → pending command
    QMap<uint16_t, PendingCmd> m_pending;
};
