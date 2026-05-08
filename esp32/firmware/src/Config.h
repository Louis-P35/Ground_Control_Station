#pragma once
#include <cstdint>

// ---------------------------------------------------------------------------
// Config — all compile-time constants for the ESP32 firmware.
// Change WiFi credentials and GCS IP here before flashing.
// ---------------------------------------------------------------------------

namespace Config {

// ── WiFi ─────────────────────────────────────────────────────────────────────
constexpr const char* WIFI_SSID     = "YOUR_SSID";
constexpr const char* WIFI_PASSWORD = "YOUR_PASSWORD";

// ── GCS UDP endpoint ──────────────────────────────────────────────────────────
constexpr const char* GCS_IP    = "192.168.1.100"; // IP of the machine running the GCS
constexpr uint16_t    GCS_PORT  = 5005;
constexpr uint16_t    LOCAL_PORT = 5005;            // port we listen on for incoming commands

// ── SPI slave pins (connected to flight controller — FC is master) ────────────
// These match the HSPI default pins on ESP32.
constexpr int PIN_MOSI = 13;
constexpr int PIN_MISO = 12;
constexpr int PIN_SCK  = 14;
constexpr int PIN_CS   = 27; // custom CS (not the HSPI default GPIO 15)

// ── Timing ────────────────────────────────────────────────────────────────────
constexpr uint32_t WIFI_RECONNECT_INTERVAL_MS = 5000; // time between reconnect attempts
constexpr uint32_t SERIAL_STATUS_INTERVAL_MS  = 1000; // serial heartbeat period

} // namespace Config
