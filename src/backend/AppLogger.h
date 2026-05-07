#pragma once
#include <QString>

// ---------------------------------------------------------------------------
// AppLogger — lightweight singleton log writer.
//
// Writes timestamped lines to <Documents>/gcs_app.log in append mode.
// Thread-safe: can be called from both the UI thread and the network thread.
//
// Log only significant events (connect, camera, parser errors, commands).
// Never log per-packet telemetry — 100 Hz would fill the disk in minutes.
//
// Usage:
//   AppLogger::init();          // once, before any other call
//   AppLogger::info("...");
//   AppLogger::warn("...");
//   AppLogger::error("...");
//   AppLogger::close();         // once, at shutdown
// ---------------------------------------------------------------------------

class AppLogger {
public:
    // Open (or create) the log file. Must be called once from main() before
    // the network thread starts so the file handle is ready.
    static void init();

    // Flush and close the log file. Call after QApplication::exec() returns.
    static void close();

    static void info (const QString& msg);
    static void warn (const QString& msg);
    static void error(const QString& msg);

private:
    AppLogger() = delete;
};
