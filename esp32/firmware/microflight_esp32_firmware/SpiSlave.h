#pragma once
#include <cstdint>
#include <driver/spi_slave.h>
#include "SpiFrame.h"

// ---------------------------------------------------------------------------
// SpiSlave — non-blocking SPI slave driver.
//
// The Flight Controller (FC) is the SPI master. It initiates every transaction
// and clocks exactly SPI_FRAME_SIZE (256) bytes in both directions simultaneously.
//
// Pins (from requirements §3.1):
//   MOSI → GPIO 13
//   MISO → GPIO 12
//   SCK  → GPIO 14
//   CS   → GPIO 27
//
// Usage in loop():
//   g_spi.update();
//   if (g_spi.newAttitude()) { /* use g_spi.attitude() */ }
//
// To forward a GCS command to the FC, call setPendingCommand(). The command
// is clocked out on the next SPI transaction, then automatically cleared.
// ---------------------------------------------------------------------------

class SpiSlave
{
public:
    void begin();  // Call once in setup() — allocates DMA buffers, inits hardware
    bool update(); // Call every loop() — non-blocking; returns true when a frame was parsed

    // ── Freshness flags — set on successful parse, cleared at next update() ───
    bool newAttitude()  const { return m_newAttitude; }
    bool newStatus()    const { return m_newStatus;   }
    bool newPid()       const { return m_newPid;      }
    bool newCalib()     const { return m_newCalib;    }
    bool newLog()       const { return m_newLog;      }

    // ── Latest telemetry snapshots ───────────────────────────────────────────
    const SpiPayloadAttitude&    attitude() const { return m_attitude; }
    const SpiPayloadStatus&      status()   const { return m_status;   }
    const SpiPayloadPid&         pid()      const { return m_pid;      }
    const SpiPayloadCalibStatus& calib()    const { return m_calib;    }
    const SpiPayloadLog&         log()      const { return m_log;      }

    // ── Command forwarding (GCS → FC via MISO) ───────────────────────────────
    void setPendingCommand(uint8_t cmdType, const uint8_t* packetBytes, size_t len);
    bool hasPendingCommand() const { return m_hasPendingCmd; }

private:
    void            queueNextTransaction();
    bool            parseRxFrame(const uint8_t* buf);
    void            buildTxFrame();
    static uint16_t crc16(const uint8_t* data, size_t len);

    // DMA-capable buffers — required by the ESP-IDF SPI slave driver
    uint8_t* m_rxBuf = nullptr;
    uint8_t* m_txBuf = nullptr;

    spi_slave_transaction_t m_trans       = {};
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
