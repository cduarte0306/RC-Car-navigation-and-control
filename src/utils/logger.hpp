#ifndef LOGGING_HPP
#define LOGGING_HPP

#include <cstdarg>


class Logger {
public:
    Logger() {}
    ~Logger() {}
    
    static Logger* getLoggerInst(void);
    void log(int logLvl, const char* format, ...);
public:
    enum {
        LOG_LVL_INFO,
        LOG_LVL_WARN,
        LOG_LVL_ERROR,
        LOG_LVL_DEBUG
    };

private:
    
};

#endif