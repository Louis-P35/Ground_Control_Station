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
        Serial.println("[SPI] FATAL: DMA buffer allocation failed — halting");
        while (true) { delay(1000); }
    }
    memset(m_rxBuf, 0, SPI_FRAME_SIZE);

    spi_bus_config_t bus = {};
    bus.mosi_io_num     = PIN_MOSI;
    bus.miso_io_num     = PIN_MISO;
    bus.sclk_io_num     = PIN_SCK;
    bus.quadwp_io_num   = -1;
    bus.quadhd_io_num   = -1;
    bus.max_transfer_sz = SPI_FRAME_SIZE;

    // mode = 0 → CPOL=0, CPHA=0 — must match the FC SPI master configuration
    spi_slave_interface_config_t slv = {};
    slv.mode         = 0;
    slv.spics_io_num = PIN_CS;
    slv.queue_size   = 1; // one pending transaction at a time is sufficient
    slv.flags        = 0;

    // SPI2_HOST = HSPI; its default pins match our wiring (MOSI=13, MISO=12, CLK=14)
    esp_err_t err = spi_slave_initialize(SPI2_HOST, &bus, &slv, SPI_DMA_CH_AUTO);
    if (err != ESP_OK)
    {
        Serial.printf("[SPI] FATAL: init failed (%s) — halting\n", esp_err_to_name(err));
        while (true) { delay(1000); }
    }

    buildTxFrame();        // fill MISO with "no command pending"
    queueNextTransaction();
    Serial.println("[SPI] Slave ready — HSPI mode 0, 256-byte frames");
    Serial.printf("[SPI] MOSI=%d  MISO=%d  SCK=%d  CS=%d\n",
                  PIN_MOSI, PIN_MISO, PIN_SCK, PIN_CS);
}

// ---------------------------------------------------------------------------
// update — non-blocking poll; returns true when a frame was successfully parsed
// ---------------------------------------------------------------------------

bool SpiSlave::update()
{
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
        Serial.printf("[SPI] get_trans_result error: %s\n", esp_err_to_name(err));
        m_transQueued = false;
        return false;
    }

    m_transQueued = false;

    // Clear all freshness flags before parsing; only the new frame's type will be set
    m_newAttitude = m_newStatus = m_newPid = m_newCalib = m_newLog = false;

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
        Serial.printf("[SPI] queue_trans failed: %s\n", esp_err_to_name(err));
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

        default:
            return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// buildTxFrame — fill m_txBuf with the MISO frame (command or idle)
// The pending command is cleared here: once this frame is clocked out the FC
// has received it, so we must not send it again on the next transaction.
// ---------------------------------------------------------------------------

void SpiSlave::buildTxFrame()
{
    memset(m_txBuf, 0, SPI_FRAME_SIZE);
    auto* frame = reinterpret_cast<SpiTxFrame*>(m_txBuf);

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
}

// ---------------------------------------------------------------------------
// setPendingCommand — store a GCS command to forward to the FC via MISO
// ---------------------------------------------------------------------------

void SpiSlave::setPendingCommand(uint8_t cmdType, const uint8_t* packetBytes, size_t len)
{
    if (len > sizeof(m_pendingCmdBuf))
    {
        Serial.printf("[SPI] setPendingCommand: payload too large (%u > %u)\n",
                      len, sizeof(m_pendingCmdBuf));
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
