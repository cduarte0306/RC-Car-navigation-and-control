#pragma once

#include <thread>


class GPSInterface {
public:
    GPSInterface();
    ~GPSInterface();

public:
    // Example method to get current coordinates
    void getCoordinates(double& latitude, double& longitude) {
        latitude = coordinates.latitudeDegrees + (coordinates.latitudeMinutes / 60.0);
        longitude = coordinates.longitudeDegrees + (coordinates.longitudeMinutes / 60.0);
    }

protected:
    typedef struct {
        float latitudeDegrees;
        float latitudeMinutes;
        float longitudeDegrees;
        float longitudeMinutes;
    } Coordinates;
protected:

    int openPort(const char* devPath,
                  int baud = 9600,
                  bool nonBlocking = false);
    void gpsInterface(void);
    int readData(char* pBuf, size_t maxLength);

    int parseIncomingData(char* pBuf, size_t length);

protected:
    std::thread gpsThread_;  // Thread for GPS data processing
    bool threadCanRun_ = true;  // Control flag for the GPS thread

    Coordinates coordinates = {0.0, 0.0, 0.0, 0.0};

    char buf[256];
    int fd = -1;  // File descriptor for the GPS device
};