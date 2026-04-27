#include <chrono>
#include <string>
#include <sstream>
#include <iomanip>
#include <cstdarg>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <cstring>
#include <systemd/sd-journal.h>
#include <syslog.h>
#include <iostream>
#include <ctime>

#include "logger.hpp"


#define INFO_PREPEND "[INFO]"
#define WARN_PREPEND "[WARN]"
#define ERR_PREPEND  "[ERROR]"


Logger* logInstance = nullptr;

Logger* Logger::getLoggerInst(void) {
    if (!logInstance) {
        logInstance = new Logger();
        
        // Open the log file
        
    }

    return logInstance;
}


void Logger::log(int logLvl, const char* format, ...) {
    char buffer[1024];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    std::time_t currentTime = std::time(nullptr);
    std::tm* localTime = std::localtime(&currentTime);
    std::string ts = std::asctime(localTime);
    ts.pop_back();
    int level;
    const char* prepend = nullptr;

    switch (logLvl) {
        case Logger::LOG_LVL_INFO:
            level = LOG_INFO;
            prepend = INFO_PREPEND;
            break;

        case Logger::LOG_LVL_WARN:
            level = LOG_WARNING;
            prepend = WARN_PREPEND;
            break;

        case Logger::LOG_LVL_ERROR:
            level = LOG_ERR;
            prepend = ERR_PREPEND;
            break;

        case Logger::LOG_LVL_DEBUG:
            return;
            break;
        
        default:
            break;
    }

    std::string message = ts + " " + prepend + " " + buffer;
    sd_journal_print(level, "%s", message.c_str());
    printf("%s", message.c_str());
}