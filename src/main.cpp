#include <QApplication>
#include "ui/MainWindow.h"

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    app.setApplicationName("Ground Control Station");
    app.setOrganizationName("Louis-P35");

    MainWindow w;
    w.show();

    return app.exec();
}
