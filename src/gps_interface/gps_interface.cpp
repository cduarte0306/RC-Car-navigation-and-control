#include "gps_interface.hpp"
#include <iostream>
#include <cstring>
#include <stdexcept>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <vector>
#include <sstream>

#define SENTENCE_HEADER "GNRMC"
#define DELIMITER ";"


GPSInterface::GPSInterface(char* devPath, int baud) {
    // Initialize GPS interface
    if (openPort(devPath, baud, /*nonBlocking=*/false) < 0) {
        throw std::runtime_error("Failed to open GPS interface");
    }
}


GPSInterface::~GPSInterface() {
    // Stop the GPS thread
    threadCanRun_ = false;
    if (gpsThread_.joinable()) {
        gpsThread_.join();
    }
}


int GPSInterface::parseIncomingData(char* pBuf, size_t length) {
    if(!pBuf) {
        return -1;
    }
    
    auto asteriskPos = std::string(pBuf).find('*');
    if (asteriskPos == std::string::npos) {
        return -2;
    }

    std::string messageChecksum = std::string(pBuf).substr(asteriskPos + 1, 2);
    auto startChar =  std::string(pBuf).find('$');
    if (startChar == std::string::npos) {
        return -2;
    }

    std::string s = std::string(pBuf).substr(startChar + 1, asteriskPos - startChar - 1);
    
    if (s.length() > length) {
        return -2;
    }

    // Calculate checksum
    unsigned char checksum = 0;
    for (char c : s) {
        checksum ^= static_cast<unsigned char>(c);
    }

    std::stringstream ss_checksum;
    ss_checksum << std::uppercase << std::hex << (checksum & 0xFF);
    std::string checksumStr = ss_checksum.str();
    if (checksumStr.length() == 1) {
        checksumStr = "0" + checksumStr;
    }

    if (checksumStr != messageChecksum) {
        return -2;
    }

    std::vector<std::string> fields;
    std::stringstream ss(s);
    std::string field;
    while (std::getline(ss, field, ',')) {
        fields.push_back(field);
    }

    if (fields[0] != SENTENCE_HEADER) {
        return 0;
    }

    // Extract latitude (ddmm.mmmm)
    std::string latitudeField = fields[3];
    std::string latitudeDir = fields[4];
    std::string longitudeField = fields[5];
    std::string longitudeDir = fields[6];

    std::string latitudeDegrees = latitudeField.substr(0, 2);
    std::string latitudeMinutes = latitudeField.substr(2);

    std::string longitudeDegrees = longitudeField.substr(0, 3);
    std::string longitudeMinutes = longitudeField.substr(3);

    try {
        double dlatitudeDegrees = std::stof(latitudeDegrees);
        double dlatitudeMinutes = std::stof(latitudeMinutes);
        double dlongitudeDegrees = std::stof(longitudeDegrees);
        double dlongitudeMinutes = std::stof(longitudeMinutes);

        // Convert to decimal degrees
        coordinates.latitudeDegrees  = dlatitudeDegrees + (dlatitudeMinutes / 60.0);
        coordinates.longitudeDegrees = dlongitudeDegrees + (dlongitudeMinutes / 60.0);

        // Apply sign based on N/S/E/W
        if (latitudeDir == "S") coordinates.latitudeDegrees = -coordinates.latitudeDegrees;
        if (longitudeDir == "W") coordinates.longitudeDegrees = -coordinates.longitudeDegrees;

        // Store or return lat/lon as needed
    } catch (const std::exception& e) {
        std::cerr << "Error parsing coordinates: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}


/**
 * @brief GPS interface thread to read GPS data
 * 
 * This method runs in a separate thread and continuously reads GPS data from the device.
 * It can be extended to parse the GPS data and update the latitude and longitude attributes.
 */
int GPSInterface::gpsDoInterface(double& latitude, double& longitude) {
    // This method would contain the logic to read GPS data from the device
    // For now, we will just simulate reading coordinates
    int n = read(fd, buf, sizeof(buf));
    if (n < 0) {
        perror("read");
    }

    int ret = parseIncomingData(buf, n);
    if (ret < 0) {
        switch(ret) {
            case -1:
                // std::cerr << "ERROR: Parsing incoming data." << std::endl;
                break;
            case -2:
                // std::cerr << "ERROR: Bad CRC" << std::endl;
                break;
            default:
                std::cerr << "Unrecognized error, code: " << ret << std::endl;
        }

        memset(buf, 0, n);  // Clear the buffer for the next read
        return ret;
    }

    memset(buf, 0, n);  // Clear the buffer for the next read

    latitude = coordinates.latitudeDegrees;
    longitude = coordinates.longitudeDegrees;

    return 0;
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
    int flags = O_RDWR | O_NOCTTY;
    if (nonBlocking) flags |= O_NONBLOCK;
    fd = ::open(devPath, flags);
    if (fd < 0 && errno == EINTR) {
        // Retry if interrupted by signal
        fd = ::open(devPath, flags);
    }
    if (fd < 0) { perror("open"); return -1; }

    termios tty{};
    if (tcgetattr(fd, &tty) != 0) { perror("tcgetattr"); return -1; }

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, baud);  // set baud rate
    cfsetospeed(&options, baud);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;  // no parity
    options.c_cflag &= ~CSTOPB;  // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;      // 8 data bits

    if (tcsetattr(fd, TCSANOW, &options) != 0) { perror("tcsetattr"); return -1; }
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
    ssize_t n = ::read(fd, pBuf, maxLength);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0; // nothing yet
        perror("read"); return -1;
    }
    return static_cast<int>(n);

}