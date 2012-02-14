TARGET = GPS
QT += network
QT -= gui
HEADERS += GPSListener.h
SOURCES += GPSListener.cpp  main.cpp
FORMS += 
RESOURCES += 
# Pour Qt-4.8.x
QMAKE_CXXFLAGS += -DQT_BUILD_CORE_LIB
