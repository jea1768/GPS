#ifndef STICKLISTENER_H_
#define GPSLISTENER_H_
#include <QtCore>
#include <QtNetwork>
#include <QTimer>
#include <iostream>

#define BAUD_RATE B4800
#define NMEA_BUF_SIZE 80

#define FAILURE_COM_TRIGGER_DELAY  	1000 	// msec
#define FAILURE_READ_TRIGGER_DELAY 	3000	// msec
#define RETRY_DELAY 				5		// sec
#define FIRST_RETRY_DELAY 			5		// sec
#define UNKNOWN_SENTENCE_LEVEL	50

#define UDP_PORT_GPS_DATA_BROADCAST 12345 //54321

class GPSListener  : public QObject
{
	Q_OBJECT
private:
    int gpsDev;
    char gps_device[64];
    int initOK;
	QSocketNotifier *socketNotifier;
	QUdpSocket *udpNmeaSocket;
	QTimer *timerGpsCommunicationFailure;
	int unknownSentence;
	QString udpGPS;
	int udpGPSReadyToSendRMC, udpGPSReadyToSendGGA;

public:
	GPSListener(const char *device);
	int initGPS(const char *device);
	void cleanCloseEverything();
	virtual ~GPSListener();

private slots:
	int readGPS(int device);
	void triggerGpsFailure();
};

#endif /* GPSLISTENER_H_ */
