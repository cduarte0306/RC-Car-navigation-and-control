#pragma once

#include <thread>


class GPSInterface {
public:
    GPSInterface();
    ~GPSInterface();

public:
    // Example method to get current coordinates
    virtual void getCoordinates(double& latitude, double& longitude) const = 0;

protected:
    // Placeholder for GPS data, can be extended with more attributes
    double latitude_ = 0.0;
    double longitude_ = 0.0;

    int openPort(const char* devPath,
                  int baud = 9600,
                  bool nonBlocking = false);
    void gpsInterface(void);
    int readData(char* pBuf, size_t maxLength);

protected:
    std::thread gpsThread_;  // Thread for GPS data processing
    bool threadCanRun_ = true;  // Control flag for the GPS thread

    int fd_ = -1;  // File descriptor for the GPS device
};