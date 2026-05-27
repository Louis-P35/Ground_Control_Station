#include <WiFi.h>
#include <WiFiUdp.h>
#include <cstring>

#include "../../../common/Protocol.h"
#include "Secrets.h"      // WIFI_SSID, WIFI_PASSWORD — not versioned
#include "SpiSlave.h"
#include "Mtf01Reader.h"
#include "SbusReader.h"

// ---------------------------------------------------------------------------
// Network configuration
// ---------------------------------------------------------------------------

static const char* STA_SSID     = WIFI_SSID;
static const char* STA_PASSWORD = WIFI_PASSWORD;

static const char* AP_SSID     = "MicroFlight-ESP32";
static const char* AP_PASSWORD = "microflight";

static const uint32_t WIFI_TIMEOUT_MS = 10000;
static const uint16_t GCS_PORT        = 5005;
static const uint16_t LOCAL_PORT      = 5005;

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------

static SpiSlave    g_spi;
static Mtf01Reader g_mtf01;
static SbusReader  g_sbus;
static WiFiUDP     g_udp;
static IPAddress   g_broadcastIp;

// Per-type sequence counters (one per PKT_* type, indexed by type byte)
static uint16_t g_seqs[0x21] = {};

// SPI frame counters reset every second for the status report
static uint32_t g_lastStatusMs = 0;
static uint32_t g_cntAttitude  = 0;
static uint32_t g_cntStatus    = 0;
static uint32_t g_cntOther     = 0;

// ---------------------------------------------------------------------------
// crc16 — CRC-16/CCITT, same polynomial as the GCS
// ---------------------------------------------------------------------------

static uint16_t crc16(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; ++b)
        {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
        }
    }
    return crc;
}

// ---------------------------------------------------------------------------
// sendPacket — common UDP send helper
// ---------------------------------------------------------------------------

static void sendPacket(const uint8_t* data, size_t len)
{
    g_udp.beginPacket(g_broadcastIp, GCS_PORT);
    g_udp.write(data, len);
    g_udp.endPacket();
}

// ---------------------------------------------------------------------------
// fillHeader — fill a PacketHeader with the standard fields
// ---------------------------------------------------------------------------

static void fillHeader(PacketHeader& hdr, uint8_t type, uint16_t payloadLen)
{
    hdr.magic        = PACKET_MAGIC;
    hdr.version      = PACKET_VERSION;
    hdr.type         = type;
    hdr.timestamp_us = (uint32_t)(millis() * 1000UL);
    hdr.seq          = ++g_seqs[type];
    hdr.payload_len  = payloadLen;
}

// ---------------------------------------------------------------------------
// sendAttitude — forward a SpiPayloadAttitude as PKT_ATTITUDE to the GCS
// ---------------------------------------------------------------------------

static void sendAttitude(const SpiPayloadAttitude& d)
{
    PktAttitude pkt{};
    fillHeader(pkt.header, PKT_ATTITUDE,
               sizeof(PktAttitude) - sizeof(PacketHeader) - sizeof(uint16_t));
    pkt.qw = d.qw; pkt.qx = d.qx; pkt.qy = d.qy; pkt.qz = d.qz;
    pkt.gx = d.gx; pkt.gy = d.gy; pkt.gz = d.gz;
    pkt.ax = d.ax; pkt.ay = d.ay; pkt.az = d.az;
    pkt.crc = crc16(reinterpret_cast<const uint8_t*>(&pkt),
                    sizeof(PacketHeader) + pkt.header.payload_len);
    sendPacket(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

// ---------------------------------------------------------------------------
// sendMtf01 — forward MTF-01 data as PKT_MTF01 to the GCS
// ---------------------------------------------------------------------------

static void sendMtf01(const Mtf01Reader& mtf)
{
    PktMtf01 pkt{};
    fillHeader(pkt.header, PKT_MTF01,
               sizeof(PktMtf01) - sizeof(PacketHeader) - sizeof(uint16_t));
    pkt.distance_m = mtf.distance();
    pkt.flow_x     = static_cast<float>(mtf.flowX());
    pkt.flow_y     = static_cast<float>(mtf.flowY());
    pkt.quality    = mtf.quality();
    pkt.crc = crc16(reinterpret_cast<const uint8_t*>(&pkt),
                    sizeof(PacketHeader) + pkt.header.payload_len);
    sendPacket(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

// ---------------------------------------------------------------------------
// sendFullStatus — forward a real SpiPayloadStatus as PKT_STATUS to the GCS.
// All battery, motor, and state fields are copied from the FC SPI frame.
// The WiFi RSSI is added by the ESP32 itself (the FC doesn't know it).
// ---------------------------------------------------------------------------

static void sendFullStatus(const SpiPayloadStatus& s)
{
    PktStatus pkt{};
    fillHeader(pkt.header, PKT_STATUS,
               sizeof(PktStatus) - sizeof(PacketHeader) - sizeof(uint16_t));

    pkt.battery_voltage = s.battery_voltage;
    pkt.battery_current = s.battery_current;
    pkt.battery_percent = s.battery_percent;
    strncpy(pkt.state, s.state, sizeof(pkt.state) - 1);
    memcpy(pkt.motor_percent, s.motor_percent, sizeof(pkt.motor_percent));

    // Map WiFi RSSI from dBm (e.g. -70) to 0–100
    int rssiDbm = WiFi.RSSI();
    pkt.wifi_rssi = (uint8_t)max(0, min(100, (rssiDbm + 100) * 2));

    pkt.crc = crc16(reinterpret_cast<const uint8_t*>(&pkt),
                    sizeof(PacketHeader) + pkt.header.payload_len);
    sendPacket(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

// ---------------------------------------------------------------------------
// sendStatus — broadcast a PKT_STATUS heartbeat to keep the GCS "connected"
// when no FC is linked via SPI. State and WiFi RSSI only; battery fields = 0.
// ---------------------------------------------------------------------------

static void sendStatus(const char* state)
{
    PktStatus pkt{};
    fillHeader(pkt.header, PKT_STATUS,
               sizeof(PktStatus) - sizeof(PacketHeader) - sizeof(uint16_t));
    strncpy(pkt.state, state, sizeof(pkt.state) - 1);
    // Map WiFi RSSI from dBm (e.g. -70) to 0–100
    int rssiDbm = WiFi.RSSI();
    pkt.wifi_rssi = (uint8_t)max(0, min(100, (rssiDbm + 100) * 2));
    pkt.crc = crc16(reinterpret_cast<const uint8_t*>(&pkt),
                    sizeof(PacketHeader) + pkt.header.payload_len);
    sendPacket(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

// ---------------------------------------------------------------------------
// sendLog — broadcast a PKT_LOG packet to the GCS
// ---------------------------------------------------------------------------

static void sendLog(const char* msg, uint8_t level = 1 /* INFO */)
{
    PktLog pkt{};
    fillHeader(pkt.header, PKT_LOG,
               sizeof(PktLog) - sizeof(PacketHeader) - sizeof(uint16_t));
    pkt.level = level;
    strncpy(pkt.text, msg, sizeof(pkt.text) - 1);
    pkt.crc = crc16(reinterpret_cast<const uint8_t*>(&pkt),
                    sizeof(PacketHeader) + pkt.header.payload_len);
    sendPacket(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

// ---------------------------------------------------------------------------
// sendRadio — forward S.Bus channel data as PKT_RADIO to the GCS.
// Raw 11-bit values (typical range 172–1811) are mapped to 1000–2000 µs,
// the standard PWM range expected by the GCS. Channels 0–7 are forwarded.
// ---------------------------------------------------------------------------

static void sendRadio(const SbusReader& sbus)
{
    PktRadio pkt{};
    fillHeader(pkt.header, PKT_RADIO,
               sizeof(PktRadio) - sizeof(PacketHeader) - sizeof(uint16_t));

    // Map SBUS raw values (172–1811 typical) to 1000–2000 µs, clamped
    for (int i = 0; i < 8; ++i)
    {
        int raw = sbus.channel(i);
        int us  = 1000 + ((raw - SBUS_RAW_MIN) * 1000) / (SBUS_RAW_MAX - SBUS_RAW_MIN);
        pkt.channels[i] = (uint16_t)max(1000, min(2000, us));
    }

    // RSSI: 0 on failsafe (signal lost), 255 otherwise
    pkt.rssi = sbus.failsafe() ? 0 : 255;

    pkt.crc = crc16(reinterpret_cast<const uint8_t*>(&pkt),
                    sizeof(PacketHeader) + pkt.header.payload_len);
    sendPacket(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

// ---------------------------------------------------------------------------
// setup
// ---------------------------------------------------------------------------

void setup()
{
    Serial.begin(115200);
    delay(200);
    Serial.println("\n========================================");
    Serial.println("  MicroFlight ESP32 — booting");
    Serial.println("========================================");

    // ── WiFi ─────────────────────────────────────────────────────────────────
    Serial.printf("[WiFi] Connecting to \"%s\"", STA_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(STA_SSID, STA_PASSWORD);

    const uint32_t deadline = millis() + WIFI_TIMEOUT_MS;
    while (WiFi.status() != WL_CONNECTED && millis() < deadline)
    {
        delay(250);
        Serial.print('.');
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED)
    {
        IPAddress ip   = WiFi.localIP();
        IPAddress mask = WiFi.subnetMask();
        g_broadcastIp  = IPAddress(
            (ip[0] & mask[0]) | (~mask[0] & 0xFF),
            (ip[1] & mask[1]) | (~mask[1] & 0xFF),
            (ip[2] & mask[2]) | (~mask[2] & 0xFF),
            0xFF
        );
        Serial.printf("[WiFi] STA  IP: %s  RSSI: %d dBm\n",
                      ip.toString().c_str(), WiFi.RSSI());
    }
    else
    {
        Serial.println("[WiFi] STA timeout — starting AP");
        WiFi.mode(WIFI_AP);
        WiFi.softAP(AP_SSID, AP_PASSWORD);
        g_broadcastIp = IPAddress(192, 168, 4, 255);
        Serial.printf("[WiFi] AP  SSID: \"%s\"  IP: %s\n",
                      AP_SSID, WiFi.softAPIP().toString().c_str());
    }

    g_udp.begin(LOCAL_PORT);
    Serial.printf("[UDP] Socket open — broadcasting to %s:%u\n",
                  g_broadcastIp.toString().c_str(), GCS_PORT);

    // ── SPI slave ─────────────────────────────────────────────────────────────
    g_spi.begin();

    // ── MTF-01 (Serial1, GPIO 25 RX / 26 TX, 115200 baud) ───────────────────
    g_mtf01.begin(Serial1, 115200, 25, 26);
    Serial.println("[MTF01] Reader started — Serial1 RX=25 TX=26 @ 115200");

    // ── S.Bus receiver (Serial2, GPIO 16 RX / 17 TX, 100000 baud 8E2 inv.) ──
    g_sbus.begin(Serial2, 16, 17);

    sendLog("[ESP32] Boot complete — SPI slave + MTF-01 + S.Bus active");
    Serial.println("[MAIN] All subsystems ready");
}

// ---------------------------------------------------------------------------
// loop
// ---------------------------------------------------------------------------

void loop()
{
    // ── MTF-01: drain UART RX buffer and parse MAVLink frames ────────────────
    g_mtf01.update();
    if (g_mtf01.newData())
    {
        sendMtf01(g_mtf01);
    }

    // ── S.Bus: drain UART RX buffer and decode frames ────────────────────────
    g_sbus.update();
    if (g_sbus.newFrame())
    {
        sendRadio(g_sbus);
    }

    // ── SPI: drain one completed transaction per call ─────────────────────────
    if (g_spi.update())
    {
        if (g_spi.newAttitude())
        {
            sendAttitude(g_spi.attitude());
            g_cntAttitude++;
        }
        if (g_spi.newStatus())
        {
            // Forward all fields from the FC: battery voltage, current,
            // percent, motor throttles, FSM state. WiFi RSSI is added here.
            sendFullStatus(g_spi.status());
            g_cntStatus++;
        }
        if (g_spi.newLog())
        {
            sendLog(g_spi.log().text, g_spi.log().level);
            g_cntOther++;
        }
    }

    // ── 1 Hz status report — serial + GCS log ────────────────────────────────
    const uint32_t now = millis();
    if (now - g_lastStatusMs >= 1000)
    {
        g_lastStatusMs = now;

        // Serial report
        Serial.printf("[STATUS] WiFi: %s  RSSI: %d dBm  heap: %u B"
                      "  SPI att=%lu  sta=%lu  other=%lu\n",
                      WiFi.status() == WL_CONNECTED
                          ? WiFi.localIP().toString().c_str()
                          : "disconnected",
                      WiFi.RSSI(),
                      ESP.getFreeHeap(),
                      g_cntAttitude, g_cntStatus, g_cntOther);

        // PKT_STATUS — fallback heartbeat so the GCS stays "connected" when the
        // FC sends no status of its own.  Skip it when the FC already forwarded
        // at least one real status frame this second to avoid overwriting it.
        if (g_cntStatus == 0)
        {
            const char* fcState = g_spi.isInitialized() ? "NO-FC" : "SPI-ERR";
            sendStatus(fcState);
        }

        // PKT_LOG — SPI throughput stats visible in the GCS terminal
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "[SPI] att=%lu/s  sta=%lu/s  other=%lu/s",
                 g_cntAttitude, g_cntStatus, g_cntOther);
        sendLog(msg);

        g_cntAttitude = g_cntStatus = g_cntOther = 0;
    }
}
