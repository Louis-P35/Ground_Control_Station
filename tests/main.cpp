#include <QCoreApplication>
#include "backend/AppLogger.h"

// Forward declarations — each test file defines its runner function.
int TestPacketParser_run  (int argc, char** argv);
int TestCommandSender_run (int argc, char** argv);
int TestTelemetryState_run(int argc, char** argv);

int main(int argc, char* argv[]) {
    QCoreApplication app(argc, argv);

    // AppLogger is used by PacketParser and CommandSender internally.
    // Initialise it so the log infrastructure is ready before the first test.
    AppLogger::init();

    int status = 0;
    status |= TestPacketParser_run  (argc, argv);
    status |= TestCommandSender_run (argc, argv);
    status |= TestTelemetryState_run(argc, argv);

    AppLogger::close();
    return status;
}
