#include "SpiSlave.h"
#include "Config.h"
#include <cstring>
#include <esp_heap_caps.h>
#include <Arduino.h>

// ---------------------------------------------------------------------------
// begin — configure hardware and queue the first SPI transaction.
// ---------------------------------------------------------------------------

void SpiSlave::begin() {
    // DMA-capable buffers are required by the ESP-IDF SPI slave driver.
    m_rxBuf = static_cast<uint8_t*>(heap_caps_malloc(SPI_FRAME_SIZE, MALLOC_CAP_DMA));
    m_txBuf = static_cast<uint8_t*>(heap_caps_malloc(SPI_FRAME_SIZE, MALLOC_CAP_DMA));
    if (!m_rxBuf || !m_txBuf) {
        Serial.println("[SPI] FATAL: DMA buffer allocation failed — halting");
        while (true) delay(1000);
    }
    memset(m_rxBuf, 0, SPI_FRAME_SIZE);

    // Bus configuration — pins must match the hardware wiring
    spi_bus_config_t bus = {};
    bus.mosi_io_num     = Config::PIN_MOSI;
    bus.miso_io_num     = Config::PIN_MISO;
    bus.sclk_io_num     = Config::PIN_SCK;
    bus.quadwp_io_num   = -1;
    bus.quadhd_io_num   = -1;
    bus.max_transfer_sz = SPI_FRAME_SIZE;

    // Slave configuration
    // mode = 0 → CPOL=0, CPHA=0 — must match the FC SPI master configuration
    spi_slave_interface_config_t slv = {};
    slv.mode         = 0;
    slv.spics_io_num = Config::PIN_CS;
    slv.queue_size   = 1; // one pending transaction at a time is enough
    slv.flags        = 0;

    // SPI2_HOST = HSPI; its default pins (MOSI=13, MISO=12, CLK=14) match our wiring
    esp_err_t err = spi_slave_initialize(SPI2_HOST, &bus, &slv, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        Serial.printf("[SPI] FATAL: init failed (%s) — halting\n", esp_err_to_name(err));
        while (true) delay(1000);
    }

    buildTxFrame(); // prepare a "no command pending" MISO frame
    queueNextTransaction();
    Serial.println("[SPI] Slave initialized (HSPI, mode 0, 256-byte frames)");
}

// ---------------------------------------------------------------------------
// update — non-blocking poll. Returns true on a successful frame parse.
// ---------------------------------------------------------------------------

bool SpiSlave::update() {
    if (!m_transQueued) {
        buildTxFrame();
        queueNextTransaction();
        return false;
    }

    // Check whether the master has completed a transaction (timeout = 0 → non-blocking)
    spi_slave_transaction_t* done = nullptr;
    esp_err_t err = spi_slave_get_trans_result(SPI2_HOST, &done, 0 /*ticks*/);
    if (err == ESP_ERR_TIMEOUT) {
        return false; // master hasn't initiated a transaction yet
    }
    if (err != ESP_OK) {
        Serial.printf("[SPI] get_trans_result error: %s\n", esp_err_to_name(err));
        m_transQueued = false;
        return false;
    }

    m_transQueued = false;

    // Clear freshness flags before parsing so only the new frame's type is flagged
    m_newAttitude = m_newStatus = m_newPid = m_newCalib = m_newLog = false;

    bool parsed = parseRxFrame(m_rxBuf);

    // Rebuild MISO frame (clears pending command if it was consumed) and queue next
    buildTxFrame();
    queueNextTransaction();

    return parsed;
}

// ---------------------------------------------------------------------------
// queueNextTransaction — submit a new SPI_FRAME_SIZE transaction.
// ---------------------------------------------------------------------------

void SpiSlave::queueNextTransaction() {
    memset(&m_trans, 0, sizeof(m_trans));
    m_trans.length    = SPI_FRAME_SIZE * 8; // length in bits
    m_trans.rx_buffer = m_rxBuf;
    m_trans.tx_buffer = m_txBuf;

    // timeout = 0: non-blocking enqueue. Fails if the queue is already full
    // (queue_size=1 and one transaction is already waiting for the master).
    esp_err_t err = spi_slave_queue_trans(SPI2_HOST, &m_trans, 0 /*ticks*/);
    m_transQueued = (err == ESP_OK);
    if (!m_transQueued) {
        Serial.printf("[SPI] queue_trans failed: %s\n", esp_err_to_name(err));
    }
}

// ---------------------------------------------------------------------------
// parseRxFrame — validate and dispatch one received SPI frame.
// ---------------------------------------------------------------------------

bool SpiSlave::parseRxFrame(const uint8_t* buf) {
    const auto* frame = reinterpret_cast<const SpiRxFrame*>(buf);

    if (frame->header.magic != SPI_MAGIC_FC_TO_ESP)
        return false;

    // Verify CRC over header + declared payload
    const size_t crcCoverage = sizeof(SpiFrameHeader) + frame->header.payload_len;
    if (crcCoverage + sizeof(uint16_t) > SPI_FRAME_SIZE)
        return false; // payload_len claims more bytes than the frame holds

    uint16_t computed = crc16(buf, crcCoverage);
    uint16_t received;
    memcpy(&received, buf + crcCoverage, sizeof(uint16_t));
    if (computed != received)
        return false;

    switch (frame->header.type) {
        case SpiFrameType::Attitude:
            if (frame->header.payload_len < sizeof(SpiPayloadAttitude)) break;
            memcpy(&m_attitude, &frame->payload.attitude, sizeof(m_attitude));
            m_newAttitude = true;
            break;

        case SpiFrameType::Status:
            if (frame->header.payload_len < sizeof(SpiPayloadStatus)) break;
            memcpy(&m_status, &frame->payload.status, sizeof(m_status));
            m_status.state[sizeof(m_status.state) - 1] = '\0'; // null-termination safety
            m_newStatus = true;
            break;

        case SpiFrameType::Pid:
            if (frame->header.payload_len < sizeof(SpiPayloadPid)) break;
            memcpy(&m_pid, &frame->payload.pid, sizeof(m_pid));
            m_newPid = true;
            break;

        case SpiFrameType::CalibStatus:
            if (frame->header.payload_len < sizeof(SpiPayloadCalibStatus)) break;
            memcpy(&m_calib, &frame->payload.calib, sizeof(m_calib));
            m_calib.message[sizeof(m_calib.message) - 1] = '\0';
            m_newCalib = true;
            break;

        case SpiFrameType::Log:
            if (frame->header.payload_len < sizeof(SpiPayloadLog)) break;
            memcpy(&m_log, &frame->payload.log, sizeof(m_log));
            m_log.text[sizeof(m_log.text) - 1] = '\0';
            m_newLog = true;
            break;

        default:
            return false; // unknown frame type
    }
    return true;
}

// ---------------------------------------------------------------------------
// buildTxFrame — fill m_txBuf with the MISO frame (pending command or idle).
// Consuming the pending command is done here: if has_cmd=1, the FC will read
// it on this transaction, so we clear the flag immediately after building.
// ---------------------------------------------------------------------------

void SpiSlave::buildTxFrame() {
    memset(m_txBuf, 0, SPI_FRAME_SIZE);
    auto* frame = reinterpret_cast<SpiTxFrame*>(m_txBuf);

    frame->magic    = SPI_MAGIC_ESP_TO_FC;
    frame->has_cmd  = m_hasPendingCmd ? 1 : 0;
    frame->cmd_type = m_pendingCmdType;

    if (m_hasPendingCmd) {
        memcpy(frame->cmd.set_pid_raw, m_pendingCmdBuf, m_pendingCmdLen);
        m_hasPendingCmd = false; // command will be clocked out on the next transaction
    }

    // CRC covers magic + has_cmd + cmd_type + the full cmd union (fixed 27 bytes)
    const size_t crcCoverage = 2 + 1 + 1 + sizeof(PktSetPid);
    frame->crc = crc16(m_txBuf, crcCoverage);
}

// ---------------------------------------------------------------------------
// setPendingCommand — store a GCS command to be forwarded to the FC.
// ---------------------------------------------------------------------------

void SpiSlave::setPendingCommand(uint8_t cmdType, const uint8_t* packetBytes, size_t len) {
    if (len > sizeof(m_pendingCmdBuf)) {
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
// crc16 — CRC-16/CCITT (same polynomial as the GCS and Protocol.h)
// ---------------------------------------------------------------------------

uint16_t SpiSlave::crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}
