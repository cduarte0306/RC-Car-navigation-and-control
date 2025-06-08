#include "gps_interface.hpp"
#include <iostream>
#include <cstring>
#include <stdexcept>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>


GPSInterface::GPSInterface() {
    // Initialize GPS interface
    if (openPort("/dev/ttyUSB0", B9600, /*nonBlocking=*/true) < 0) {
        throw std::runtime_error("Failed to open GPS interface");
    }
    
    this->gpsThread_ = std::thread(&GPSInterface::gpsInterface, this);
}


GPSInterface::~GPSInterface() {
    // Stop the GPS thread
    threadCanRun_ = false;
    if (gpsThread_.joinable()) {
        gpsThread_.join();
    }
}


/**
 * @brief GPS interface thread to read GPS data
 * 
 * This method runs in a separate thread and continuously reads GPS data from the device.
 * It can be extended to parse the GPS data and update the latitude and longitude attributes.
 */
void GPSInterface::gpsInterface(void) {
    // This method would contain the logic to read GPS data from the device
    // For now, we will just simulate reading coordinates
    while (threadCanRun_) {
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}


/**
 * @brief Open the GPS device port
 * 
 * @param devPath Path to the GPS device
 * @param baud Baud rate for the serial communication
 * @param nonBlocking If true, open the port in non-blocking mode
 * @return int 0 on success, -1 on failure
 */
int GPSInterface::openPort(const char* devPath, int baud, bool nonBlocking) {
    int flags = O_RDWR | O_NOCTTY | O_SYNC;
    if (nonBlocking) flags |= O_NONBLOCK;
    fd_ = ::open(devPath, flags);
    if (fd_ < 0) { perror("open"); return -1; }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) { perror("tcgetattr"); return -1; }

    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;   // 8 data bits
    tty.c_cflag |= CLOCAL | CREAD;                // ignore modem, enable RX
    tty.c_cflag &= ~(PARENB | PARODD);            // no parity
    tty.c_cflag &= ~CSTOPB;                       // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                      // no HW flow-ctrl

    tty.c_iflag = 0;                              // raw input
    tty.c_oflag = 0;                              // raw output
    tty.c_lflag = 0;                              // raw mode

    // read returns as soon as 1 byte arrives or after 100 ms
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) { perror("tcsetattr"); return -1; }
    return 0;
}


/**
 * @brief Read data from the GPS device
 * 
 * @param pBuf Pointer to the buffer where data will be stored
 * @param maxLength Maximum length of data to read
 * @return int Number of bytes read, or -1 on error
 */
int GPSInterface::readData(char* pBuf, size_t maxLength) {
    ssize_t n = ::read(fd_, pBuf, maxLength);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0; // nothing yet
        perror("read"); return -1;
    }
    return static_cast<int>(n);

}