#pragma once
#include <cstdint>
#include <driver/spi_slave.h>
#include "SpiFrame.h"
#include "Protocol.h"

// ---------------------------------------------------------------------------
// SpiSlave — non-blocking SPI slave driver.
//
// The Flight Controller (FC) is the SPI master. It initiates every transaction
// and clocks exactly SPI_FRAME_SIZE bytes in both directions simultaneously.
//
// Usage in loop():
//   spiSlave.update();         // drains completed transactions, queues next
//   if (spiSlave.newAttitude()) doSomethingWith(spiSlave.attitude());
//
// When a GCS command is received via UDP, call setPendingCommand() to load it
// into the MISO buffer. It will be sent to the FC on the next SPI transaction.
// ---------------------------------------------------------------------------

class SpiSlave {
public:
    // Initialize SPI slave hardware and queue the first transaction.
    void begin();

    // Non-blocking update. Call every loop() iteration.
    // Returns true if a complete SPI frame was successfully received and parsed.
    bool update();

    // ── Freshness flags ───────────────────────────────────────────────────────
    // Set to true when new data arrives from the FC; cleared at next update().
    bool newAttitude()  const { return m_newAttitude; }
    bool newStatus()    const { return m_newStatus;   }
    bool newPid()       const { return m_newPid;      }
    bool newCalib()     const { return m_newCalib;    }
    bool newLog()       const { return m_newLog;      }

    // ── Latest telemetry (read by UdpLink) ───────────────────────────────────
    const SpiPayloadAttitude&    attitude() const { return m_attitude; }
    const SpiPayloadStatus&      status()   const { return m_status;   }
    const SpiPayloadPid&         pid()      const { return m_pid;      }
    const SpiPayloadCalibStatus& calib()    const { return m_calib;    }
    const SpiPayloadLog&         log()      const { return m_log;      }

    // ── Command forwarding (written by UdpLink, sent to FC via MISO) ─────────
    // Stores the raw packet bytes (including header and CRC) for the FC.
    // The command is consumed (cleared) once it has been clocked out via MISO.
    void setPendingCommand(uint8_t cmdType, const uint8_t* packetBytes, size_t len);
    bool hasPendingCommand() const { return m_hasPendingCmd; }

private:
    void     queueNextTransaction();
    bool     parseRxFrame(const uint8_t* buf);
    void     buildTxFrame();
    static uint16_t crc16(const uint8_t* data, size_t len);

    // DMA-capable buffers (allocated in begin() via heap_caps_malloc)
    uint8_t* m_rxBuf = nullptr;
    uint8_t* m_txBuf = nullptr;

    spi_slave_transaction_t m_trans      = {};
    bool                    m_transQueued = false;

    // ── Parsed telemetry ─────────────────────────────────────────────────────
    SpiPayloadAttitude    m_attitude = {};
    SpiPayloadStatus      m_status   = {};
    SpiPayloadPid         m_pid      = {};
    SpiPayloadCalibStatus m_calib    = {};
    SpiPayloadLog         m_log      = {};

    bool m_newAttitude = false;
    bool m_newStatus   = false;
    bool m_newPid      = false;
    bool m_newCalib    = false;
    bool m_newLog      = false;

    // ── Pending command for FC ────────────────────────────────────────────────
    bool    m_hasPendingCmd  = false;
    uint8_t m_pendingCmdType = 0;
    uint8_t m_pendingCmdBuf[sizeof(PktSetPid)] = {}; // sized to largest command
    size_t  m_pendingCmdLen  = 0;
};
