#include "SpiSlave.h"
#include <esp_heap_caps.h>
#include <Arduino.h>
#include <cstring>

// ---------------------------------------------------------------------------
// Pin assignments — §3.1 of ESP32_REQUIREMENTS.md
// ---------------------------------------------------------------------------
static constexpr int PIN_MOSI = 13;
static constexpr int PIN_MISO = 12;
static constexpr int PIN_SCK  = 14;
static constexpr int PIN_CS   = 27;

// ---------------------------------------------------------------------------
// begin — allocate DMA buffers, configure hardware, queue first transaction
// ---------------------------------------------------------------------------

void SpiSlave::begin()
{
    // DMA-capable buffers are mandatory for the ESP-IDF SPI slave driver
    m_rxBuf = static_cast<uint8_t*>(heap_caps_malloc(SPI_FRAME_SIZE, MALLOC_CAP_DMA));
    m_txBuf = static_cast<uint8_t*>(heap_caps_malloc(SPI_FRAME_SIZE, MALLOC_CAP_DMA));
    if (!m_rxBuf || !m_txBuf)
    {
        snprintf(m_pendingLog, sizeof(m_pendingLog),
                 "[SPI] ERROR: DMA buffer allocation failed — SPI disabled");
        m_hasPendingLog = true;
        return;
    }
    memset(m_rxBuf, 0, SPI_FRAME_SIZE);

    spi_bus_config_t bus = {};
    bus.mosi_io_num     = PIN_MOSI;
    bus.miso_io_num     = PIN_MISO;
    bus.sclk_io_num     = PIN_SCK;
    bus.quadwp_io_num   = -1;
    bus.quadhd_io_num   = -1;
    bus.max_transfer_sz = SPI_FRAME_SIZE;

    // mode = 3 → CPOL=1, CPHA=1 — must match the FC SPI master (SPI_POLARITY_HIGH + SPI_PHASE_2EDGE)
    spi_slave_interface_config_t slv = {};
    slv.mode         = 3;
    slv.spics_io_num = PIN_CS;
    slv.queue_size   = 1; // one pending transaction at a time is sufficient
    slv.flags        = 0;

    // SPI2_HOST = HSPI; its default pins match our wiring (MOSI=13, MISO=12, CLK=14)
    esp_err_t err = spi_slave_initialize(SPI2_HOST, &bus, &slv, SPI_DMA_CH_AUTO);
    if (err != ESP_OK)
    {
        snprintf(m_pendingLog, sizeof(m_pendingLog),
                 "[SPI] ERROR: init failed (%s) — SPI disabled", esp_err_to_name(err));
        m_hasPendingLog = true;
        return; // non-fatal: loop() continues and heartbeats are still sent
    }

    m_initialized = true;
    buildTxFrame();        // fill MISO with "no command pending"
    queueNextTransaction();
}

// ---------------------------------------------------------------------------
// update — non-blocking poll; returns true when a frame was successfully parsed
// ---------------------------------------------------------------------------

bool SpiSlave::update()
{
    if (!m_initialized)
    {
        return false;
    }

    if (!m_transQueued)
    {
        buildTxFrame();
        queueNextTransaction();
        return false;
    }

    // Timeout = 0 → returns immediately if no transaction has completed yet
    spi_slave_transaction_t* done = nullptr;
    esp_err_t err = spi_slave_get_trans_result(SPI2_HOST, &done, 0);
    if (err == ESP_ERR_TIMEOUT)
    {
        return false; // FC hasn't initiated a transaction yet — normal at idle
    }
    if (err != ESP_OK)
    {
        snprintf(m_pendingLog, sizeof(m_pendingLog),
                 "[SPI] get_trans_result error: %s", esp_err_to_name(err));
        m_hasPendingLog = true;
        m_transQueued = false;
        return false;
    }

    m_transQueued = false;

    // Clear all freshness flags before parsing; only the new frame's type will be set
    m_newAttitude = m_newStatus = m_newPid = m_newCalib = m_newLog = m_newRadio = false;
    m_newGps = m_newMtf01 = m_newMag = false;

    bool parsed = parseRxFrame(m_rxBuf);

    // Rebuild MISO (consumes pending command if any) and queue the next transaction
    buildTxFrame();
    queueNextTransaction();

    return parsed;
}

// ---------------------------------------------------------------------------
// queueNextTransaction — enqueue a new 256-byte SPI transaction (non-blocking)
// ---------------------------------------------------------------------------

void SpiSlave::queueNextTransaction()
{
    memset(&m_trans, 0, sizeof(m_trans));
    m_trans.length    = SPI_FRAME_SIZE * 8; // length in bits
    m_trans.rx_buffer = m_rxBuf;
    m_trans.tx_buffer = m_txBuf;

    esp_err_t err = spi_slave_queue_trans(SPI2_HOST, &m_trans, 0);
    m_transQueued = (err == ESP_OK);
    if (!m_transQueued)
    {
        snprintf(m_pendingLog, sizeof(m_pendingLog),
                 "[SPI] queue_trans failed: %s", esp_err_to_name(err));
        m_hasPendingLog = true;
    }
}

// ---------------------------------------------------------------------------
// parseRxFrame — validate and dispatch one 256-byte MOSI frame from the FC
// ---------------------------------------------------------------------------

bool SpiSlave::parseRxFrame(const uint8_t* buf)
{
    const auto* frame = reinterpret_cast<const SpiRxFrame*>(buf);

    if (frame->header.magic != SPI_MAGIC_FC_TO_ESP)
    {
        return false;
    }

    // Verify CRC over header + declared payload
    const size_t crcCoverage = sizeof(SpiFrameHeader) + frame->header.payload_len;
    if (crcCoverage + sizeof(uint16_t) > SPI_FRAME_SIZE)
    {
        return false; // payload_len claims more bytes than the frame holds
    }

    uint16_t computed = crc16(buf, crcCoverage);
    uint16_t received;
    memcpy(&received, buf + crcCoverage, sizeof(uint16_t));
    if (computed != received)
    {
        return false;
    }

    switch (frame->header.type)
    {
        case SpiFrameType::Attitude:
            if (frame->header.payload_len < sizeof(SpiPayloadAttitude))
            {
                break;
            }
            memcpy(&m_attitude, &frame->payload.attitude, sizeof(m_attitude));
            m_newAttitude = true;
            break;

        case SpiFrameType::Status:
            if (frame->header.payload_len < sizeof(SpiPayloadStatus))
            {
                // Log once so the GCS can show the mismatch without spamming
                if (!m_statusSizeMismatchLogged)
                {
                    snprintf(m_pendingLog, sizeof(m_pendingLog),
                             "[SPI] Status rejected: len=%u expected>=%u",
                             (unsigned)frame->header.payload_len,
                             (unsigned)sizeof(SpiPayloadStatus));
                    m_hasPendingLog           = true;
                    m_statusSizeMismatchLogged = true;
                }
                break;
            }
            memcpy(&m_status, &frame->payload.status, sizeof(m_status));
            m_status.state[sizeof(m_status.state) - 1] = '\0';
            m_newStatus = true;
            break;

        case SpiFrameType::Pid:
            if (frame->header.payload_len < sizeof(SpiPayloadPid))
            {
                break;
            }
            memcpy(&m_pid, &frame->payload.pid, sizeof(m_pid));
            m_newPid = true;
            break;

        case SpiFrameType::CalibStatus:
            if (frame->header.payload_len < sizeof(SpiPayloadCalibStatus))
            {
                break;
            }
            memcpy(&m_calib, &frame->payload.calib, sizeof(m_calib));
            m_calib.message[sizeof(m_calib.message) - 1] = '\0';
            m_newCalib = true;
            break;

        case SpiFrameType::Log:
            if (frame->header.payload_len < sizeof(SpiPayloadLog))
            {
                break;
            }
            memcpy(&m_log, &frame->payload.log, sizeof(m_log));
            m_log.text[sizeof(m_log.text) - 1] = '\0';
            m_newLog = true;
            break;

        case SpiFrameType::Radio:
            if (frame->header.payload_len < sizeof(SpiPayloadRadio))
            {
                break;
            }
            memcpy(&m_radio, &frame->payload.radio, sizeof(m_radio));
            m_newRadio = true;
            break;

        case SpiFrameType::Gps:
            if (frame->header.payload_len < sizeof(SpiPayloadGps))
            {
                break;
            }
            memcpy(&m_gps, &frame->payload.gps, sizeof(m_gps));
            m_newGps = true;
            break;

        case SpiFrameType::Mtf01:
            if (frame->header.payload_len < sizeof(SpiPayloadMtf01))
            {
                break;
            }
            memcpy(&m_mtf01, &frame->payload.mtf01, sizeof(m_mtf01));
            m_newMtf01 = true;
            break;

        case SpiFrameType::Mag:
            if (frame->header.payload_len < sizeof(SpiPayloadMag))
            {
                break;
            }
            memcpy(&m_mag, &frame->payload.mag, sizeof(m_mag));
            m_newMag = true;
            break;

        default:
            return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// buildTxFrame — fill m_txBuf with the MISO frame.
//
// The frame has two independent sections:
//   1. Command section [0..32]: GCS command forwarding, CRC-protected.
//      The pending command is cleared here so it is not re-sent.
//   2. S.Bus section  [33..65]: raw receiver values for the FC.
//      These are refreshed every call and stay valid until the next frame.
// ---------------------------------------------------------------------------

void SpiSlave::buildTxFrame()
{
    memset(m_txBuf, 0, SPI_FRAME_SIZE);
    auto* frame = reinterpret_cast<SpiTxFrame*>(m_txBuf);

    // ── Command section ───────────────────────────────────────────────────────
    frame->magic    = SPI_MAGIC_ESP_TO_FC;
    frame->has_cmd  = m_hasPendingCmd ? 1 : 0;
    frame->cmd_type = m_pendingCmdType;

    if (m_hasPendingCmd)
    {
        memcpy(frame->cmd.set_pid_raw, m_pendingCmdBuf, m_pendingCmdLen);
        m_hasPendingCmd = false; // command will be clocked out on this transaction
    }

    // CRC covers magic(2) + has_cmd(1) + cmd_type(1) + full cmd union(27)
    const size_t crcCoverage = 2 + 1 + 1 + sizeof(PktSetPid);
    frame->crc = crc16(m_txBuf, crcCoverage);

    // ── S.Bus section ─────────────────────────────────────────────────────────
    frame->has_sbus = m_hasSbus ? 1 : 0;
    if (m_hasSbus)
    {
        memcpy(frame->sbus_raw, m_sbusRaw, sizeof(frame->sbus_raw));
    }

    // ── GPS section ───────────────────────────────────────────────────────────
    frame->has_gps = m_hasGps ? 1 : 0;
    if (m_hasGps)
    {
        memcpy(&frame->gps, &m_gpsRaw, sizeof(frame->gps));
    }

    // ── MTF-01 section ────────────────────────────────────────────────────────
    frame->has_mtf01 = m_hasMtf01 ? 1 : 0;
    if (m_hasMtf01)
    {
        memcpy(&frame->mtf01, &m_mtf01Raw, sizeof(frame->mtf01));
    }

    // ── Magnetometer section ──────────────────────────────────────────────────
    frame->has_mag = m_hasMag ? 1 : 0;
    if (m_hasMag)
    {
        memcpy(&frame->mag, &m_magRaw, sizeof(frame->mag));
    }
}

// ---------------------------------------------------------------------------
// setSbusRaw — store raw S.Bus channel values to include in the next MISO frame
// ---------------------------------------------------------------------------

void SpiSlave::setSbusRaw(const uint16_t channels[SBUS_SPI_CHANNELS], bool valid)
{
    memcpy(m_sbusRaw, channels, SBUS_SPI_CHANNELS * sizeof(uint16_t));
    m_hasSbus = valid;
}

// ---------------------------------------------------------------------------
// setGps / setMtf01 — store raw sensor readings to include in the next MISO frame
// ---------------------------------------------------------------------------

void SpiSlave::setGps(const SpiPayloadGps& gps)
{
    m_gpsRaw = gps;
    m_hasGps = true;
}

void SpiSlave::setMtf01(const SpiPayloadMtf01& mtf01)
{
    m_mtf01Raw = mtf01;
    m_hasMtf01 = true;
}

void SpiSlave::setMag(const SpiPayloadMag& mag)
{
    m_magRaw = mag;
    m_hasMag = true;
}

// ---------------------------------------------------------------------------
// setPendingCommand — store a GCS command to forward to the FC via MISO
// ---------------------------------------------------------------------------

void SpiSlave::setPendingCommand(uint8_t cmdType, const uint8_t* packetBytes, size_t len)
{
    if (len > sizeof(m_pendingCmdBuf))
    {
        snprintf(m_pendingLog, sizeof(m_pendingLog),
                 "[SPI] setPendingCommand: payload too large (%u > %u)",
                 (unsigned)len, (unsigned)sizeof(m_pendingCmdBuf));
        m_hasPendingLog = true;
        return;
    }
    m_pendingCmdType = cmdType;
    memcpy(m_pendingCmdBuf, packetBytes, len);
    m_pendingCmdLen  = len;
    m_hasPendingCmd  = true;
}

// ---------------------------------------------------------------------------
// crc16 — CRC-16/CCITT (same polynomial as the GCS)
// ---------------------------------------------------------------------------

uint16_t SpiSlave::crc16(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b)
        {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
        }
    }
    return crc;
}
