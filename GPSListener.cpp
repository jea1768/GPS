
#ifdef __linux__
#include <linux/types.h>
#include <linux/ioctl.h>
#include <error.h>
#endif

#ifdef __OpenBSD__
#include <sys/types.h>
#include <sys/ioctl.h>
#endif

#include <iostream>
#include <termios.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include "GPSListener.h"

GPSListener::GPSListener(const char *device)
{
	// !!! Reglages ttyUSBx !!!
	// stty -F /dev/ttyUSBx speed 4800 cs8 -cstopb -parenb
	// verifier avec cat < /dev/ttyUSBx

   int initStatus = initGPS(device);
   while (initStatus == -1 )
   {
	 std::cout << "INIT GPS DEVICE FAILED, RETRYING IN " << FIRST_RETRY_DELAY << " SEC.\n";
	 sleep(FIRST_RETRY_DELAY);
	 initStatus = initGPS(device);
   }
}

int GPSListener::initGPS(const char *device)
{
  struct termios newtio;
  int e;

  initOK = 0;

  //std::cout << "GPS DEVICE=[" << device << "]\n";
  strncpy(gps_device, device, strlen(device)+1);
  std::cout << "INIT GPS DEVICE=[" << gps_device << "]\n";

	    //http://dce.felk.cvut.cz/pos/cv5/doc/serial.html
		gpsDev = open(gps_device, O_RDONLY|O_NDELAY|O_NOCTTY); // pas de difference
		if( gpsDev < 0 )
		{
		 initOK = 0;
		 perror("Error on fd ");
		 return -1;
		}
		initOK = 1;
		std::cout << "INIT OK FOR DEVICE=[" << gps_device << "]\n";
		// save current port settings
		tcgetattr(gpsDev,&newtio);

#ifdef __linux__
#pragma message "#### LINUX ####"
	    newtio.c_cflag = BAUD_RATE | CS8 | CREAD | CLOCAL;  // CS8=8bit,no parity,1 stop bit - CLOCAL=ignore modem status lines
	    e = tcsetattr(gpsDev, TCSANOW, &newtio);
	    if( e < 0 )
	    {
	    	perror("Error in tcsetattr ");
	    	return -1;
	    }
#endif

#ifdef __OpenBSD__
#pragma message "#### OPENBSD ####"
	    e = cfsetspeed(&newtio,BAUD_RATE); // for OpenBSD, the speed must be set by cfsetspeed
	    if( e < 0 )
	    {
	    	perror("Error SetSpeed ");
	    	exit(-1);
	    }
	    newtio.c_cflag = CS8 | CREAD | CLOCAL;  // CS8=8bit,no parity,1 stop bit - CLOCAL=ignore modem status lines
	    e = tcsetattr(gpsDev,TCSANOW,&newtio);
	    if( e < 0 )
	    {
	    	perror("Error in tcsetattr ");
	    	exit(-1);
	    }
#endif
	    e = tcflush(gpsDev,TCIFLUSH);
	    if( e < 0 )
	    {
	    	perror("Error Flush ");
	    	exit(-1);
	    }

	   udpNmeaSocket = new QUdpSocket(this);
	   unknownSentence = 0;
	   udpGPSReadyToSendRMC = udpGPSReadyToSendGGA = 0;

       // http://doc.qt.nokia.com/latest/qsocketnotifier.html#details
       socketNotifier = new QSocketNotifier(gpsDev, QSocketNotifier::Read, this); // 100% CPU ?! is ::Write
       connect(socketNotifier, SIGNAL(activated(int)), this, SLOT(readGPS(int)));

      // Chien de garde deconnection/defaillance GPS (typiquement deconnection)
       timerGpsCommunicationFailure = new QTimer(this);
       connect(timerGpsCommunicationFailure, SIGNAL(timeout()), this, SLOT(triggerGpsFailure()));

       std::cout << "GPS Listener connected, END INIT\n";

      return 0;
}

int GPSListener::readGPS(int gps)
{

  socketNotifier->setEnabled(false); // Qt instruction: disable pendant le traitement - voir url
  unknownSentence++;

  // dans x seconde sera declenche le traitement de la defaillance
  timerGpsCommunicationFailure->start(FAILURE_READ_TRIGGER_DELAY); // sauf si tout Ok, alors timer redemarre en fin de fonction

  if ( unknownSentence > UNKNOWN_SENTENCE_LEVEL )
  {
	  socketNotifier->setEnabled(false);
	  return -1;
  }

	char nmea_buf[NMEA_BUF_SIZE];
	int status = read(gps, nmea_buf, NMEA_BUF_SIZE);
	if ( status < 0 )
	{
	  perror("GPS file descriptor !");
	  socketNotifier->setEnabled(false);
	  return -1;
	}
    nmea_buf[NMEA_BUF_SIZE-1] = '\0';

	if ( nmea_buf[0] == 10 )
	{
	    timerGpsCommunicationFailure->start(FAILURE_COM_TRIGGER_DELAY);
		socketNotifier->setEnabled(true);
		return 0;
	}

	for(int i=0; i < NMEA_BUF_SIZE; i++)
	 if ( nmea_buf[i] ==  10 ) nmea_buf[i] = '\0';

	//printf("nmea Buffer = %s\n",nmea_buf);

	// RMC [UTC],[A=ok,V=warning],[lat],N,[lon],E,[speed Knots],[course],[date],[magnetic variation],E|W,[checksum]
     QString nmea= QString(nmea_buf);
     if ( nmea.indexOf("$GPRMC", 0, Qt::CaseInsensitive) == 0)
     {
    	 unknownSentence = 0;
    	 QStringList nmeaValues = nmea.split(",");
    	 QString utc    = nmeaValues[1];
    	  if (  utc == "" ) utc = "---";
    	 QString ok     = nmeaValues[2];
    	 QString lat    = nmeaValues[3];
    	  if (  lat == "" ) lat = "---";
    	 QString latNS  = nmeaValues[4];
    	  //if (  latNS == "" ) latNS = "---";
    	 QString lon    = nmeaValues[5];
    	  if ( lon == "" ) lon = "---";
    	 QString lonEW  = nmeaValues[6];
    	  //if (  lonEW == "" ) lonEW = "---";
    	 QString speed  = nmeaValues[7];
    	  if (  speed == "" ) speed = "0";
    	 QString course = nmeaValues[8];
    	  if ( course == "" ) course = "---";
    	 QString date   = nmeaValues[9];
    	  if (  date == "" ) date = "---";
    	 QString magvar = nmeaValues[10];
    	  if (  magvar== "" ) magvar = "---";
    	 if ( nmeaValues.length() > 12 )  // !!??!! magEW n'existe pas sur le BL323 mais bien sur le ND-100
    	 {
    	  QString magEW  = nmeaValues[11];
     	  if (  magEW == "" ) magEW = "---";
    	  QString check  = nmeaValues[12];
     	  if (  check == "" ) check = "---";
    	 }
    	 else
    	 {
       	  QString check  = nmeaValues[11];
     	  if ( check == "" ) check = "---";
    	 }
    	 double kmh = speed.toDouble() * 1.852; // conversion nautical miles -> km/h
    	 QString kmperhour = QString("%1").arg(kmh, 0, 'f', 0); // 3 = field width
         // end RMC

    	 QString utcReformated; // met en forme l'heure
    	 utcReformated = utc.mid(0,2)+":"+utc.mid(2,2)+":"+utc.mid(4,2);

    	 if ( lon.startsWith("00", Qt::CaseInsensitive) )
    	  lon = lon.remove(0,2);

    	 //std::cout << "RMC UTC=" << utcReformated.toStdString() << " OK=" << ok.toStdString() << " LAT=" << lat.toStdString() << latNS.toStdString() << " LON=" << lon.toStdString() << lonEW.toStdString() << " SPEED=" << kmperhour.toStdString() << "\n";

    	 if ( !udpGPS.isEmpty() ) udpGPS += "&";
    	 udpGPS = udpGPS + "UTC="+utcReformated+"&OK="+ok+"&LAT="+lat+latNS+"&LON="+lon+lonEW+"&SPE="+kmperhour+"&HEA="+course;
     	 udpGPSReadyToSendRMC = 1;

    	 timerGpsCommunicationFailure->start(FAILURE_COM_TRIGGER_DELAY);
     }

 	// GGA [UTC],[lat],N,[lon],E,[fix],[satellites],[dilution],[altitude],M,[heightgeoid],M,[Timelast update DGPS],[checksum]
    // Fix: 0 = invalid, 1 = GPS, 2 = DGPS
      if ( nmea.indexOf("$GPGGA", 0, Qt::CaseInsensitive) == 0)
      {
     	 unknownSentence = 0;
     	 QStringList nmeaValues = nmea.split(",");
     	 QString utc    = nmeaValues[1];
     	  if ( utc == "" ) utc = "---";
     	 QString lat    = nmeaValues[2];
     	  if (  lat == "" ) lat  = "---";
     	 QString latNS  = nmeaValues[3];
    	  //if ( latNS == "" ) latNS = "---";
     	 QString lon    = nmeaValues[4];
    	  if (  lon == "" ) lon = "---";
     	 QString lonEW  = nmeaValues[5];
    	  //if ( lonEW == "" ) lonEW = "---";
     	 QString fix  	= nmeaValues[6];
    	  if (  fix == "" ) fix = "---";
     	 QString sat	= nmeaValues[7];
    	  if (  sat == "" ) sat = "---";
     	 QString dilut  = nmeaValues[8];
    	  if (  dilut == "" ) dilut = "---";
     	 QString alt	= nmeaValues[9];
     	  if ( alt == "" ) alt = "---";
     	 QString geoid	= nmeaValues[11];
     	  if (  geoid == "" ) geoid = "---";
         // end GGA
     	 //std::cout << "GGA UTC=" << utc.toStdString() << " LAT=" << lat.toStdString() << latNS.toStdString() << " LON=" << lon.toStdString() << lonEW.toStdString() << " FIX=" << fix.toStdString() << " SAT=" << sat.toStdString()<< " DILUTION=" << dilut.toStdString() << " ALT=" << alt.toStdString() << " GEOID=" << geoid.toStdString() << "\n";

    	 if ( !udpGPS.isEmpty() ) udpGPS += "&";
     	 udpGPS = udpGPS + "FIX="+fix+"&SAT="+sat+"&ALT="+alt+"&DIL="+dilut+"&GEO="+geoid;
     	 udpGPSReadyToSendGGA = 1;

     	 timerGpsCommunicationFailure->start(FAILURE_COM_TRIGGER_DELAY);
      }

       if ( (udpGPSReadyToSendRMC) == 1 && (udpGPSReadyToSendGGA == 1)  )
       {
    	   std::cout << "GPS = " << udpGPS.toStdString() << "\n";
             // Envoie UDP
         	 QByteArray datagram = QByteArray(udpGPS.toAscii());
         	 //udpNmeaSocket->joinMulticastGroup(QHostAddress::Broadcast);
         	 udpNmeaSocket->writeDatagram(datagram.data(), datagram.size(),QHostAddress::Broadcast, UDP_PORT_GPS_DATA_BROADCAST);
         udpGPS = "";
         udpGPSReadyToSendRMC = udpGPSReadyToSendGGA = 0;
       }


     socketNotifier->setEnabled(true);
    return 0;
}

void GPSListener::cleanCloseEverything()
{
    // nettoyage de tout ce qui a ete cree dans initGPS
    socketNotifier->setEnabled(false);
	close(gpsDev);
	disconnect(socketNotifier);
	disconnect(udpNmeaSocket);
	disconnect(timerGpsCommunicationFailure);
	free(socketNotifier);
	free(udpNmeaSocket);
	free(timerGpsCommunicationFailure);
}
void GPSListener::triggerGpsFailure()
{
	// arret du watchdog
	timerGpsCommunicationFailure->stop();
    if ( initOK == 1 )
    {
     std::cout << "\nInit etait Ok, nettoyage complet\n";
     cleanCloseEverything(); // Init etait OK,
    }
    else // Init pas OK, juste le watchdog a nettoyer, le reste n'a pas ete cree
    {
        std::cout << "\nDefaillance Init, nettoyage watchdog\n";
    	close(gpsDev);
    	disconnect(timerGpsCommunicationFailure);
    	free(timerGpsCommunicationFailure);
    }

	std::cout << "TIMER TRIGGER, GPS FAILURE !, Retrying in " << FIRST_RETRY_DELAY << " seconds...\n";

	sleep(FIRST_RETRY_DELAY); // Tous les Timers sont arretes grace aux lignes ci-dessus, on met ce que l'on veut en delai

	std::cout << "Reinit " << gps_device << "\n";
	int status =  initGPS(gps_device);
	while (  status < 0 )
    {
        std::cout << "(loop) TIMER TRIGGER, GPS FAILURE !, Retrying in " << RETRY_DELAY << " seconds...\n";
    	sleep(RETRY_DELAY);
    	std::cout << "Reinit " << gps_device << "\n";
    	status =  initGPS(gps_device);
    }
    //exit(-1);
}

GPSListener::~GPSListener()
{
}


