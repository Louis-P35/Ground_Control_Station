#include <WiFi.h>
#include <WiFiUdp.h>
#include <cstring>

#include "../../../common/Protocol.h"
#include "Secrets.h"      // WIFI_SSID, WIFI_PASSWORD — not versioned
#include "SpiSlave.h"
#include "Mtf01Reader.h"
#include "SbusReader.h"
#include "GpsReader.h"
#include "MagReader.h"
#include "Bmp180Reader.h"

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

// I2C pins for the BN-880 on-board compass (HMC5883L / QMC5883L)
static const int MAG_SDA_PIN = 21;
static const int MAG_SCL_PIN = 22;

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------

static SpiSlave    g_spi;
static Mtf01Reader g_mtf01;
static SbusReader  g_sbus;
static GpsReader   g_gps;
static MagReader   g_mag;
static Bmp180Reader g_baro;
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
// sendMtf01 — forward MTF-01 data (echoed back by the FC over SPI) as
// PKT_MTF01 to the GCS. The ESP32 no longer relays its own UART reading
// directly: the value travels ESP32 → FC (MISO) → FC → ESP32 (MOSI), exactly
// like the S.Bus → processed-radio round trip.
// ---------------------------------------------------------------------------

static void sendMtf01(const SpiPayloadMtf01& m)
{
    PktMtf01 pkt{};
    fillHeader(pkt.header, PKT_MTF01,
               sizeof(PktMtf01) - sizeof(PacketHeader) - sizeof(uint16_t));
    pkt.distance_m = m.distance_m;
    pkt.flow_x     = static_cast<float>(m.flow_x);
    pkt.flow_y     = static_cast<float>(m.flow_y);
    pkt.quality    = m.quality;
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
// sendRadio — forward processed radio data (received from FC via SPI) as
// PKT_RADIO to the GCS. The FC applies calibration and mixing; the values
// are already in standard PWM microseconds (1000–2000 µs).
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// sendGps — forward a GPS fix (echoed back by the FC over SPI) as PKT_GPS to
// the GCS. Same round trip as sendMtf01(): the ESP32 reads the BN-880, hands
// the fix to the FC over MISO, and relays whatever the FC sends back on MOSI.
// ---------------------------------------------------------------------------

static void sendGps(const SpiPayloadGps& g)
{
    PktGps pkt{};
    fillHeader(pkt.header, PKT_GPS,
               sizeof(PktGps) - sizeof(PacketHeader) - sizeof(uint16_t));
    pkt.latitude    = g.latitude;
    pkt.longitude   = g.longitude;
    pkt.altitude_m  = g.altitude_m;
    pkt.speed_ms    = g.speed_ms;
    pkt.heading_deg = g.heading_deg;
    pkt.satellites  = g.satellites;
    pkt.fix_type    = g.fix_type;
    pkt.crc = crc16(reinterpret_cast<const uint8_t*>(&pkt),
                    sizeof(PacketHeader) + pkt.header.payload_len);
    sendPacket(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

// ---------------------------------------------------------------------------
// sendRadio — forward processed radio data (received from FC via SPI) as
// PKT_RADIO to the GCS. The FC applies calibration and mixing; the values
// are already in standard PWM microseconds (1000–2000 µs).
// ---------------------------------------------------------------------------

static void sendRadio(const SpiPayloadRadio& r)
{
    PktRadio pkt{};
    fillHeader(pkt.header, PKT_RADIO,
               sizeof(PktRadio) - sizeof(PacketHeader) - sizeof(uint16_t));

    // Forward the first 8 channels to the GCS (PktRadio carries channels[8])
    for (int i = 0; i < 8; ++i)
    {
        pkt.channels[i] = r.channels[i];
    }
    pkt.rssi = r.rssi;

    pkt.crc = crc16(reinterpret_cast<const uint8_t*>(&pkt),
                    sizeof(PacketHeader) + pkt.header.payload_len);
    sendPacket(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

// ---------------------------------------------------------------------------
// sendMag — forward the FC-filtered magnetometer (received via SPI) as PKT_MAG
// to the GCS, where it drives the compass widget. Same round trip as GPS.
// ---------------------------------------------------------------------------

static void sendMag(const SpiPayloadMag& m)
{
    PktMag pkt{};
    fillHeader(pkt.header, PKT_MAG,
               sizeof(PktMag) - sizeof(PacketHeader) - sizeof(uint16_t));
    pkt.x = m.x;
    pkt.y = m.y;
    pkt.z = m.z;
    pkt.crc = crc16(reinterpret_cast<const uint8_t*>(&pkt),
                    sizeof(PacketHeader) + pkt.header.payload_len);
    sendPacket(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

// ---------------------------------------------------------------------------
// sendBaro — forward a BMP180 reading straight to the GCS as PKT_BARO, where it
// drives the barometer widget.
//
// NOTE: unlike GPS / MTF-01 / magnetometer, this is a DIRECT ESP32 → GCS path
// for now — a first debug pass to confirm the I2C sensor works. It does not yet
// go through the FC over SPI. Once validated it should follow the same SPI
// round trip as the other sensors (expose the reading on MISO, relay the value
// the FC echoes back on MOSI).
// ---------------------------------------------------------------------------

static void sendBaro(float pressurePa, float temperatureC, float altitudeM)
{
    PktBaro pkt{};
    fillHeader(pkt.header, PKT_BARO,
               sizeof(PktBaro) - sizeof(PacketHeader) - sizeof(uint16_t));
    pkt.pressure_pa   = pressurePa;
    pkt.temperature_c = temperatureC;
    pkt.altitude_m    = altitudeM;
    pkt.crc = crc16(reinterpret_cast<const uint8_t*>(&pkt),
                    sizeof(PacketHeader) + pkt.header.payload_len);
    sendPacket(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

// ---------------------------------------------------------------------------
// scanI2cBus — probe every 7-bit address and log which ones ACK. Pure debug
// aid: lets us see exactly what sits on the bus (and at which address) instead
// of guessing. The BN-880 compass answers at 0x0D or 0x1E, a genuine BMP180 at
// 0x77, a BMP280 often at 0x76.
// ---------------------------------------------------------------------------

static void scanI2cBus()
{
    char    found[96] = {0};
    size_t  len = 0;
    uint8_t count = 0;

    for (uint8_t addr = 1; addr < 127; ++addr)
    {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) // device ACKed
        {
            ++count;
            len += snprintf(found + len, sizeof(found) - len, " 0x%02X", addr);
            if (len >= sizeof(found) - 6) break;
        }
    }

    char msg[128];
    if (count == 0)
        snprintf(msg, sizeof(msg), "[I2C] scan: no device responding on SDA=21 SCL=22");
    else
        snprintf(msg, sizeof(msg), "[I2C] scan: %u device(s):%s", count, found);
    sendLog(msg, count == 0 ? 3 /* ERROR */ : 1 /* INFO */);
}

// ---------------------------------------------------------------------------
// setup
// ---------------------------------------------------------------------------

void setup()
{
    // ── GPS (UART0 remapped, GPIO 4 RX / 2 TX, 9600 baud) ───────────────────
    // Serial is remapped away from USB — debug output goes to GCS via WiFi/UDP.
    g_gps.begin(Serial, 4, 2, 9600);

    // ── WiFi ─────────────────────────────────────────────────────────────────
    WiFi.mode(WIFI_STA);
    WiFi.begin(STA_SSID, STA_PASSWORD);

    const uint32_t deadline = millis() + WIFI_TIMEOUT_MS;
    while (WiFi.status() != WL_CONNECTED && millis() < deadline)
    {
        delay(250);
    }

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
    }
    else
    {
        WiFi.mode(WIFI_AP);
        WiFi.softAP(AP_SSID, AP_PASSWORD);
        g_broadcastIp = IPAddress(192, 168, 4, 255);
    }

    g_udp.begin(LOCAL_PORT);

    // Log WiFi result — UDP is now up so sendLog works from here on
    {
        char msg[128];
        if (WiFi.status() == WL_CONNECTED)
            snprintf(msg, sizeof(msg), "[WiFi] STA connected — IP: %s  RSSI: %d dBm",
                     WiFi.localIP().toString().c_str(), WiFi.RSSI());
        else
            snprintf(msg, sizeof(msg), "[WiFi] STA timeout — AP started  SSID: %s  IP: %s",
                     AP_SSID, WiFi.softAPIP().toString().c_str());
        sendLog(msg);
    }
    sendLog("[GPS] UART0 RX=4 TX=2 @ 9600 baud — BN-880 active");

    // ── SPI slave ─────────────────────────────────────────────────────────────
    g_spi.begin();
    if (!g_spi.isInitialized())
        sendLog("[SPI] ERROR: slave init failed — SPI disabled", 3 /* ERROR */);
    else
        sendLog("[SPI] Slave ready — HSPI mode 3, 256-byte frames");

    // ── MTF-01 (Serial1, GPIO 25 RX / 26 TX, 115200 baud) ───────────────────
    g_mtf01.begin(Serial1, 115200, 25, 26);
    sendLog("[MTF01] Reader started — Serial1 RX=25 TX=26 @ 115200");

    // ── S.Bus receiver (Serial2, GPIO 16 RX / 17 TX, 100000 baud 8E2 inv.) ──
    g_sbus.begin(Serial2, 16, 17);

    // ── Magnetometer (BN-880 compass, I2C SDA=21 SCL=22) ────────────────────
    Wire.begin(MAG_SDA_PIN, MAG_SCL_PIN);

    // Debug: report every address present on the bus before probing sensors.
    scanI2cBus();

    if (g_mag.begin())
    {
        char msg[64];
        snprintf(msg, sizeof(msg), "[MAG] %s detected on I2C", g_mag.chipName());
        sendLog(msg);
    }
    else
    {
        sendLog("[MAG] no compass found on I2C (QMC5883L 0x0D / HMC5883L 0x1E)",
                3 /* ERROR */);
    }

    // ── Barometer (BMP180, same I2C bus, address 0x77) ──────────────────────
    if (g_baro.begin())
    {
        sendLog("[BARO] BMP180 detected on I2C (0x77)");
    }
    else
    {
        sendLog("[BARO] no BMP180 found on I2C (0x77)", 3 /* ERROR */);
    }

    sendLog("[ESP32] Boot complete — SPI slave + MTF-01 + S.Bus + GPS + MAG + BARO active");
}

// ---------------------------------------------------------------------------
// loop
// ---------------------------------------------------------------------------

void loop()
{
    // ── GPS: drain UART RX buffer and parse NMEA frames ─────────────────────
    g_gps.update();

    // ── MTF-01: drain UART RX buffer and parse MAVLink frames ────────────────
    // The raw reading is written into the SPI MISO buffer so the FC can read it.
    // The ESP32 no longer forwards it to the GCS directly; the FC echoes the
    // value back over MOSI and we relay that (mirrors the S.Bus → radio path).
    g_mtf01.update();
    if (g_mtf01.newData())
    {
        SpiPayloadMtf01 m{};
        m.distance_m = g_mtf01.distance();
        m.flow_x     = g_mtf01.flowX();
        m.flow_y     = g_mtf01.flowY();
        m.quality    = g_mtf01.quality();
        g_spi.setMtf01(m);
    }

    // ── S.Bus: drain UART RX buffer and decode frames ────────────────────────
    // Raw values are written into the SPI MISO buffer so the FC can read them.
    // The FC applies calibration / mixing and sends back processed values.
    g_sbus.update();
    if (g_sbus.newFrame())
    {
        uint16_t rawCh[SBUS_SPI_CHANNELS];
        for (int i = 0; i < SBUS_SPI_CHANNELS; ++i)
        {
            rawCh[i] = g_sbus.channel(i);
        }
        g_spi.setSbusRaw(rawCh, !g_sbus.failsafe());

        // Debug: forward the raw receiver values straight to the GCS terminal
        // (throttled to 5 Hz). This is the value read off the S.Bus UART before
        // it goes to the FC — lets us confirm the receiver → ESP32 link works
        // independently of the FC round trip.
        static uint32_t s_lastSbusLogMs = 0;
        if (millis() - s_lastSbusLogMs >= 200)
        {
            s_lastSbusLogMs = millis();
            char msg[128];
            snprintf(msg, sizeof(msg),
                     "[SBUS] %s ch1-8: %u %u %u %u %u %u %u %u",
                     g_sbus.failsafe() ? "FAILSAFE" : "ok",
                     rawCh[0], rawCh[1], rawCh[2], rawCh[3],
                     rawCh[4], rawCh[5], rawCh[6], rawCh[7]);
            sendLog(msg);
        }
    }

    // ── Magnetometer: read at 5 Hz and write raw X/Y/Z into the SPI MISO buffer
    //    for the FC. Like GPS / MTF-01 the ESP32 no longer forwards it to the GCS
    //    directly — the FC echoes it back over MOSI and we relay that.
    static uint32_t s_lastMagMs = 0;
    if (millis() - s_lastMagMs >= 200)
    {
        s_lastMagMs = millis();
        if (g_mag.update())
        {
            SpiPayloadMag mag{};
            mag.x = g_mag.x();
            mag.y = g_mag.y();
            mag.z = g_mag.z();
            g_spi.setMag(mag);
        }
    }

    // ── Barometer: read at 5 Hz and forward straight to the GCS (PKT_BARO) ────
    //    First debug pass — direct ESP32 → GCS path, not through the FC over SPI
    //    yet. The blocking read (~9 ms) is throttled so it does not starve the
    //    100 Hz attitude relay.
    static uint32_t s_lastBaroMs = 0;
    if (millis() - s_lastBaroMs >= 200)
    {
        s_lastBaroMs = millis();
        if (g_baro.update())
        {
            sendBaro(g_baro.pressurePa(), g_baro.temperatureC(), g_baro.altitudeM());
        }
    }

    // ── SPI: drain one completed transaction per call ─────────────────────────
    g_spi.update();

    // Forward any internal diagnostic log regardless of update() result
    if (g_spi.hasPendingLog())
    {
        sendLog(g_spi.pendingLogText(), 2 /* WARN */);
        g_spi.clearPendingLog();
    }

    if (g_spi.newAttitude())
    {
        sendAttitude(g_spi.attitude());
        g_cntAttitude++;
    }
    if (g_spi.newStatus())
    {
        sendFullStatus(g_spi.status());
        g_cntStatus++;
    }
    if (g_spi.newRadio())
    {
        sendRadio(g_spi.radio());
        g_cntOther++;
    }
    if (g_spi.newGps())
    {
        sendGps(g_spi.gps());
        g_cntOther++;
    }
    if (g_spi.newMtf01())
    {
        sendMtf01(g_spi.mtf01());
        g_cntOther++;
    }
    if (g_spi.newMag())
    {
        // FC-filtered compass → PKT_MAG → GCS compass widget.
        sendMag(g_spi.mag());
        g_cntOther++;
    }
    if (g_spi.newLog())
    {
        sendLog(g_spi.log().text, g_spi.log().level);
        g_cntOther++;
    }

    // ── 1 Hz status report — serial + GCS log ────────────────────────────────
    const uint32_t now = millis();
    if (now - g_lastStatusMs >= 1000)
    {
        g_lastStatusMs = now;

        // GPS — hand the latest fix to the FC over SPI (MISO) at 1 Hz, even with
        // no fix, so the FC always has live data. The FC echoes it back on MOSI
        // and we relay that to the GCS (no direct ESP32 → GCS GPS path anymore).
        {
            SpiPayloadGps g{};
            g.latitude    = g_gps.latitude();
            g.longitude   = g_gps.longitude();
            g.altitude_m  = g_gps.altitude();
            g.speed_ms    = g_gps.speed();
            g.heading_deg = g_gps.heading();
            g.satellites  = g_gps.satellites();
            g.fix_type    = g_gps.fixType();
            g_spi.setGps(g);
            g_gps.clearFix();
        }

        // 1 Hz stats log visible in the GCS terminal
        {
            char msg[128];
            snprintf(msg, sizeof(msg),
                     "[STATUS] WiFi: %s  RSSI: %d dBm  heap: %u B"
                     "  SPI att=%lu  sta=%lu  other=%lu",
                     WiFi.status() == WL_CONNECTED
                         ? WiFi.localIP().toString().c_str()
                         : "disconnected",
                     WiFi.RSSI(),
                     ESP.getFreeHeap(),
                     g_cntAttitude, g_cntStatus, g_cntOther);
            sendLog(msg);
        }

        // PKT_STATUS — fallback heartbeat so the GCS stays "connected" when no FC
        // is present.  Only fire when NO SPI data at all arrived this second:
        // if the FC is sending attitude (100 Hz) it is clearly alive even if a
        // Status frame happened to be missed, so do not overwrite the GCS with
        // zeros and "NO-FC".
        if (g_cntStatus == 0 && g_cntAttitude == 0)
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
