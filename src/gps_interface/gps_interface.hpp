#pragma once

#include <thread>


class GPSInterface {
public:
    GPSInterface(char* devPath, int baud = 9600);
    ~GPSInterface();

public:
    typedef struct {
        float latitudeDegrees;
        float longitudeDegrees;
    } Coordinates;

    int gpsDoInterface(double& latitude, double& longitude);

protected:

    int openPort(const char* devPath,
                  int baud = 9600,
                  bool nonBlocking = false);
    int readData(char* pBuf, size_t maxLength);

    int parseIncomingData(char* pBuf, size_t length);

protected:
    std::thread gpsThread_;  // Thread for GPS data processing
    bool threadCanRun_ = true;  // Control flag for the GPS thread

    Coordinates coordinates = {0.0, 0.0};

    char buf[256];
    int fd = -1;  // File descriptor for the GPS device
};