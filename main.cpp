#include <QCoreApplication>
#include "GPSListener.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    if ( argc < 2 )
    {
    	std::cout << "Nom device manquant ! (/dev/ttyUSBx, /dev/cuaUx, ...)\n";
    	exit(-1);
    }
    GPSListener gps(argv[1]);
    return a.exec();
}
