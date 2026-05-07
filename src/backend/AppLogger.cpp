#include "AppLogger.h"
#include <QFile>
#include <QTextStream>
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
    QTextStream s(&s_file);
    s << QDateTime::currentDateTime().toString("[yyyy-MM-dd HH:mm:ss.zzz] ")
      << '[' << level << "] " << msg << '\n';
    // Flush after every write so the log is readable even if the app crashes
    s_file.flush();
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void AppLogger::init() {
    // Place logs/ next to the executable so the file is easy to find
    QString logDir = QCoreApplication::applicationDirPath() + "/logs";
    QDir().mkpath(logDir);
    QString path = logDir + "/gcs_app.log";
    s_file.setFileName(path);
    if (!s_file.open(QIODevice::Append | QIODevice::Text)) return;
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
