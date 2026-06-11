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
// Data flow:
//   FC → ESP32 (MOSI): attitude, status, PID, calib, log, processed radio,
//                      echoed GPS + MTF-01
//   ESP32 → FC (MISO): GCS commands (PID/calib) + raw S.Bus / GPS / MTF-01
//
// Usage in loop():
//   g_spi.update();
//   if (g_spi.newAttitude()) { ... }
//   if (g_spi.newRadio())    { ... }  // processed radio from FC
//   if (g_spi.newGps())      { ... }  // GPS echoed back by FC
//
// To forward raw sensor data to the FC, call setSbusRaw() / setGps() /
// setMtf01() after each new reading. To forward a GCS command, use
// setPendingCommand().
// ---------------------------------------------------------------------------

class SpiSlave
{
public:
    void begin();  // Call once in setup() — allocates DMA buffers, inits hardware
    bool update(); // Call every loop() — non-blocking; returns true when a frame was parsed
    bool isInitialized() const { return m_initialized; }

    // ── Freshness flags — set on successful parse, cleared at next update() ───
    bool newAttitude()  const { return m_newAttitude; }
    bool newStatus()    const { return m_newStatus;   }
    bool newPid()       const { return m_newPid;      }
    bool newCalib()     const { return m_newCalib;    }
    bool newLog()       const { return m_newLog;      }
    bool newRadio()     const { return m_newRadio;    }
    bool newGps()       const { return m_newGps;      }
    bool newMtf01()     const { return m_newMtf01;    }
    bool newMag()       const { return m_newMag;      }

    // ── Latest telemetry snapshots ───────────────────────────────────────────
    const SpiPayloadAttitude&    attitude() const { return m_attitude; }
    const SpiPayloadStatus&      status()   const { return m_status;   }
    const SpiPayloadPid&         pid()      const { return m_pid;      }
    const SpiPayloadCalibStatus& calib()    const { return m_calib;    }
    const SpiPayloadLog&         log()      const { return m_log;      }
    const SpiPayloadRadio&       radio()    const { return m_radio;    }
    const SpiPayloadGps&         gps()      const { return m_gps;      }
    const SpiPayloadMtf01&       mtf01()    const { return m_mtf01;    }
    const SpiPayloadMag&         mag()      const { return m_mag;      }

    // ── S.Bus raw data (MISO → FC) ───────────────────────────────────────────
    // Call after each decoded S.Bus frame. The values are included in every
    // subsequent SPI MISO frame until overwritten.
    void setSbusRaw(const uint16_t channels[SBUS_SPI_CHANNELS], bool valid);

    // ── GPS / MTF-01 raw data (MISO → FC) ────────────────────────────────────
    // Call after each new sensor reading. The value is included in every
    // subsequent SPI MISO frame until overwritten, just like setSbusRaw().
    void setGps(const SpiPayloadGps& gps);
    void setMtf01(const SpiPayloadMtf01& mtf01);
    void setMag(const SpiPayloadMag& mag);

    // ── Diagnostic log (drained by main loop into sendLog()) ─────────────────
    bool        hasPendingLog()  const { return m_hasPendingLog; }
    const char* pendingLogText() const { return m_pendingLog;    }
    void        clearPendingLog()      { m_hasPendingLog = false; }

    // ── Command forwarding (GCS → FC via MISO) ───────────────────────────────
    void setPendingCommand(uint8_t cmdType, const uint8_t* packetBytes, size_t len);
    bool hasPendingCommand() const { return m_hasPendingCmd; }

private:
    void            queueNextTransaction();
    bool            parseRxFrame(const uint8_t* buf);
    void            buildTxFrame();
    static uint16_t crc16(const uint8_t* data, size_t len);

    bool m_initialized = false;

    // DMA-capable buffers — required by the ESP-IDF SPI slave driver
    uint8_t* m_rxBuf = nullptr;
    uint8_t* m_txBuf = nullptr;

    spi_slave_transaction_t m_trans       = {};
    bool                    m_transQueued = false;

    // ── Parsed telemetry (MOSI → from FC) ───────────────────────────────────
    SpiPayloadAttitude    m_attitude = {};
    SpiPayloadStatus      m_status   = {};
    SpiPayloadPid         m_pid      = {};
    SpiPayloadCalibStatus m_calib    = {};
    SpiPayloadLog         m_log      = {};
    SpiPayloadRadio       m_radio    = {};
    SpiPayloadGps         m_gps      = {};
    SpiPayloadMtf01       m_mtf01    = {};
    SpiPayloadMag         m_mag      = {};

    bool m_newAttitude = false;
    bool m_newStatus   = false;
    bool m_newPid      = false;
    bool m_newCalib    = false;
    bool m_newLog      = false;
    bool m_newRadio    = false;
    bool m_newGps      = false;
    bool m_newMtf01    = false;
    bool m_newMag      = false;

    // ── Raw S.Bus data to include in MISO ────────────────────────────────────
    bool     m_hasSbus = false;
    uint16_t m_sbusRaw[SBUS_SPI_CHANNELS] = {};

    // ── Raw GPS / MTF-01 data to include in MISO ─────────────────────────────
    bool            m_hasGps   = false;
    SpiPayloadGps   m_gpsRaw   = {};
    bool            m_hasMtf01 = false;
    SpiPayloadMtf01 m_mtf01Raw = {};
    bool            m_hasMag   = false;
    SpiPayloadMag   m_magRaw   = {};

    // ── Diagnostic log — queued by parseRxFrame(), drained by the main loop ─
    bool m_hasPendingLog            = false;
    bool m_statusSizeMismatchLogged = false;
    char m_pendingLog[128]          = {};

    // ── Pending GCS command for FC ────────────────────────────────────────────
    bool    m_hasPendingCmd  = false;
    uint8_t m_pendingCmdType = 0;
    uint8_t m_pendingCmdBuf[sizeof(PktSetPid)] = {};
    size_t  m_pendingCmdLen  = 0;
};
