#include <QApplication>
#include "ui/MainWindow.h"
#include "backend/AppLogger.h"

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    app.setApplicationName("Ground Control Station");
    app.setOrganizationName("Louis-P35");

    AppLogger::init();

    MainWindow w;
    w.show();

    int ret = app.exec();

    AppLogger::close();
    return ret;
}
