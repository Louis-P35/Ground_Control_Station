#include "AppLogger.h"
#include <QFile>
#include <QDateTime>
#include <QDir>
#include <QCoreApplication>
#include <QMutex>
#include <QMutexLocker>

// ---------------------------------------------------------------------------
// Internal state — file-static so no global constructor order issues
// ---------------------------------------------------------------------------
static QFile  s_file;
static QMutex s_mutex;

static void writeLine(const char* level, const QString& msg) {
    QMutexLocker lock(&s_mutex);
    if (!s_file.isOpen()) return;
    // Write directly via QFile::write() to avoid QTextStream's internal buffer,
    // then flush immediately so the line is on disk even if the app crashes.
    QString line = QDateTime::currentDateTime().toString("[yyyy-MM-dd HH:mm:ss.zzz] [")
                 + level + "] " + msg + '\n';
    s_file.write(line.toUtf8());
    s_file.flush();
}

static constexpr int MAX_LOG_FILES = 10;

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void AppLogger::init() {
    QString logDir = QCoreApplication::applicationDirPath() + "/logs";
    QDir dir;
    dir.mkpath(logDir);

    // One file per session, named by launch timestamp
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd_HH-mm-ss");
    QString path = logDir + "/gcs_" + timestamp + ".log";

    // Keep only the MAX_LOG_FILES most recent files — delete the oldest if needed
    QFileInfoList files = QDir(logDir).entryInfoList(
        {"gcs_*.log"}, QDir::Files, QDir::Time | QDir::Reversed);
    // entryInfoList with QDir::Reversed + QDir::Time → oldest first
    while (files.size() >= MAX_LOG_FILES) {
        QFile::remove(files.first().absoluteFilePath());
        files.removeFirst();
    }

    s_file.setFileName(path);
    if (!s_file.open(QIODevice::WriteOnly | QIODevice::Text)) return;
    writeLine("INFO", QString("=== GCS started  (log: %1) ===").arg(path));
}

void AppLogger::close() {
    writeLine("INFO", "=== GCS stopped ===");
    QMutexLocker lock(&s_mutex);
    s_file.close();
}

void AppLogger::info (const QString& msg) { writeLine("INFO",  msg); }
void AppLogger::warn (const QString& msg) { writeLine("WARN",  msg); }
void AppLogger::error(const QString& msg) { writeLine("ERROR", msg); }
