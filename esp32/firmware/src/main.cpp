#include <Arduino.h>
#include <WiFi.h>
#include "Config.h"
#include "SpiSlave.h"
#include "UdpLink.h"

// ---------------------------------------------------------------------------
// Globals — one instance of each subsystem
// ---------------------------------------------------------------------------

static SpiSlave g_spi;
static UdpLink  g_udp;

// ---------------------------------------------------------------------------
// WiFi helpers
// ---------------------------------------------------------------------------

static void connectWifi() {
    Serial.printf("[WiFi] Connecting to \"%s\"", Config::WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASSWORD);

    // Block until connected — acceptable at boot time
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print('.');
    }
    Serial.printf("\n[WiFi] Connected  IP: %s  RSSI: %d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
}

// Non-blocking WiFi watchdog — call every loop() iteration.
// Initiates a reconnect attempt every WIFI_RECONNECT_INTERVAL_MS if the link
// is lost. WiFi.begin() is non-blocking; the connection completes asynchronously.
static void maintainWifi() {
    if (WiFi.status() == WL_CONNECTED) return;

    static uint32_t s_lastAttempt = 0;
    const uint32_t  now           = millis();
    if (now - s_lastAttempt < Config::WIFI_RECONNECT_INTERVAL_MS) return;

    s_lastAttempt = now;
    Serial.println("[WiFi] Link lost — reconnecting...");
    WiFi.disconnect(true);
    WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASSWORD);
}

// ---------------------------------------------------------------------------
// setup
// ---------------------------------------------------------------------------

void setup() {
    Serial.begin(115200);
    delay(200); // let the UART settle
    Serial.println("\n========================================");
    Serial.println("  GCS-ESP32 firmware  — booting");
    Serial.println("========================================");

    connectWifi();
    g_udp.begin(Config::LOCAL_PORT, Config::GCS_IP, Config::GCS_PORT);
    g_spi.begin();

    Serial.println("[MAIN] All subsystems ready — entering loop");
}

// ---------------------------------------------------------------------------
// loop — bare metal, ~100 Hz
// ---------------------------------------------------------------------------

static uint32_t s_lastStatusMs = 0;

void loop() {
    // Keep WiFi alive — non-blocking watchdog
    maintainWifi();

    // ── Core data path ───────────────────────────────────────────────────────
    // SpiSlave::update() drains one completed SPI transaction per call (if any)
    // and queues the next transaction immediately.
    g_spi.update();

    // UdpLink::update() forwards fresh SPI telemetry to the GCS and reads any
    // incoming commands, passing them back to g_spi for forwarding to the FC.
    g_udp.update(g_spi);

    // ── Serial heartbeat (1 Hz) ──────────────────────────────────────────────
    const uint32_t now = millis();
    if (now - s_lastStatusMs >= Config::SERIAL_STATUS_INTERVAL_MS) {
        s_lastStatusMs = now;
        Serial.printf("[STATUS] WiFi: %s  RSSI: %d dBm  heap: %u B free\n",
                      WiFi.status() == WL_CONNECTED
                          ? WiFi.localIP().toString().c_str()
                          : "disconnected",
                      WiFi.RSSI(),
                      ESP.getFreeHeap());
    }
}
