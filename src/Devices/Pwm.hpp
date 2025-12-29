#pragma once

#include "DeviceBase.hpp"

namespace Device
{  
class Pwm : public DeviceBase {
public:
    Pwm(const char* devName, int freq);
    ~Pwm();

    int writeEnable(bool status);
    int writeDutyCycle(int dutyPercent);
private:
    int doIoctl(int fd, unsigned long cmd, void *arg);

    int fd = -1;  // File descriptor
};    
} // namespace Device::


#pragma endregion
