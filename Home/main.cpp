#include "Home.h"
#include "../AutoControlDll/controlProbe.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    Home window;
    //controlProbe window;
    window.show();
    return app.exec();
}
