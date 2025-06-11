#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <string>
#include <vector>
#include <sstream>


#define SENTENCE_HEADER "$GNRMC"
#define DELIMITER ";"


typedef struct {
    float latitudeDegrees;
    float latitudeMinutes;
    float longitudeDegrees;
    float longitudeMinutes;
} Coordinates;


Coordinates coor = {
    .latitudeDegrees = 0.0f,
    .latitudeMinutes = 0.0f,
    .longitudeDegrees = 0.0f,
    .longitudeMinutes = 0.0f
};


int parseIncomingData(const char* data, size_t length) {
    // Example parsing logic, modify as needed
    if (data == nullptr) {
        return -1;
    }

    std::string s = std::string(data);
    std::vector<std::string> fields;
    std::stringstream ss(data);
    std::string field;
    while (std::getline(ss, field, ',')) {
        fields.push_back(field);
    }

    if (fields[0] != SENTENCE_HEADER) {
        return 0;
    }

    std::cout << data << std::endl;
    std::string latitudeMin, latitudeDegrees, longitudeMin, longitudeDegrees;

    latitudeDegrees = fields[3].substr(0, 2);
    latitudeMin = fields[3].substr(latitudeDegrees.length(), fields[3].length() - latitudeDegrees.length());

    longitudeDegrees = fields[5].substr(0, 3);
    longitudeMin = fields[5].substr(longitudeDegrees.length(), fields[3].length() - longitudeDegrees.length());

    coor.latitudeDegrees = std::stof(latitudeDegrees);
    coor.latitudeMinutes = std::stof(latitudeMin);
    coor.longitudeDegrees = std::stof(longitudeDegrees);
    coor.longitudeMinutes = std::stof(longitudeMin);
    std::cout << "Latitude: " << coor.latitudeDegrees << "° " << coor.latitudeMinutes << "'\n";
    std::cout << "Longitude: " << coor.longitudeDegrees << "° " << coor.longitudeMinutes << "'\n";
    // Here you can add more parsing logic as needed
    
    // Return a dummy value
    return 0;
}


int main() {
    int fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);    if (fd == -1) {
        perror("open");
        return 1;
    }

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B9600);  // set baud rate
    cfsetospeed(&options, B9600);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;  // no parity
    options.c_cflag &= ~CSTOPB;  // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;      // 8 data bits

    tcsetattr(fd, TCSANOW, &options);

    char buf[512];
    int ret = -1;

    while(true) {
        int n = read(fd, buf, sizeof(buf));
        if (n < 0) {
            perror("read");
        }

        // std::cout << buf << std::endl;

        ret = parseIncomingData(buf, n);
        if (ret < 0) {
            std::cerr << "Error parsing incoming data." << std::endl;
        }

        // Keep the program running to maintain the serial connection
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    close(fd);
    return 0;
}