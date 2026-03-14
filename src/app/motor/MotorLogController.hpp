#pragma once

#include <thread>

namespace Motor {
class MotorLogController {
public:
    MotorLogController(char* tty="/dev/ttyTHS1");
    ~MotorLogController();

private:

    /**
     * @brief Opens the TTY interface
     * 
     * @return int File descriptor of opened TTY, or -1 on error
     */
    int openInterface();

    void mainThreadFunc();

    std::thread m_LogThread;

    const char* tty = nullptr;
    const char* logFilePath = "/var/log/motor_log.log";
    int fd = -1;
    bool m_threadCanRun = true;
};
}

#pragma endregion