#include "MotorLogController.hpp"
#include "utils/logger.hpp"

#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <regex>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>


namespace Motor
{
MotorLogController::MotorLogController(char* tty) : tty(tty) {
    fd = openInterface();

    m_LogThread = std::thread(&MotorLogController::mainThreadFunc, this);
}

MotorLogController::~MotorLogController() {
    if (fd != -1) {
        close(fd);
    }
}

int MotorLogController::openInterface() {
    fd = open(this->tty, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Failed to open %s\r\n", this->tty);
        return -1;
    }

    fcntl(fd, F_SETFL, 0); // Blocking read

    struct termios options;
    tcgetattr(fd, &options);

    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    options.c_cflag |= (CLOCAL | CREAD); // Enable receiver, ignore modem control lines
    options.c_cflag &= ~PARENB;          // No parity
    options.c_cflag &= ~CSTOPB;          // 1 stop bit
    options.c_cflag &= ~CSIZE;           // Clear current char size mask
    options.c_cflag |= CS8;              // 8 data bits

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
    options.c_iflag &= ~(IXON | IXOFF | IXANY);         // No software flow control
    options.c_oflag &= ~OPOST;                          // Raw output

    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

void MotorLogController::mainThreadFunc() {

    std::regex pattern(R"(\[ *(\d+) *\] <([^>]+)> (.+))");
    const std::filesystem::path logPath(logFilePath);

    char buffer[1024];
    std::string logLine;

    while(m_threadCanRun) {
        ssize_t bytesRead = read(fd, buffer, sizeof(buffer) - 1);
        if (bytesRead < 0) {
            Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR, "Failed to read from %s\r\n", this->tty);
            continue;
        }

        buffer[bytesRead] = '\0';
        logLine += buffer;
        std::replace(logLine.begin(), logLine.end(), '\r', '\n');

        // Process complete lines only
        size_t pos;
        while ((pos = logLine.find('\n')) != std::string::npos) {
            std::string line = logLine.substr(0, pos);
            logLine.erase(0, pos + 1);

            std::smatch matches;
            if (std::regex_search(line, matches, pattern)) {
                Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "[PSoC] %s\r\n", matches[0].str().c_str());
                matches[0].str() += "\r\n";

                // Write to the log file
                if (!std::filesystem::exists(logPath)) {
                    std::ofstream file(logPath);
                    if (file.is_open()) {
                        file << line;
                        file << "\n";
                        file.close();
                    }
                } else {
                    std::ofstream file(logPath, std::ios::app);
                    if (file.is_open()) {
                        file << line;
                        file << "\n";
                        file.close();
                    }
                }
            }
        }
    }
}
} // namespace Motor
