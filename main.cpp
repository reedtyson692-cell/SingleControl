#include "mainwindow.h"
#include "EtherCatbus.h"
#include "DriverTransInterface.h"
//#include "MotionCal.h"
#include <QApplication>

//EtherCatBus stewart;
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
