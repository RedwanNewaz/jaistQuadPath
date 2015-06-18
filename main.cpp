#include "planner.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    planner w;
    w.show();

    return a.exec();
}
