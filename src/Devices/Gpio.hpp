#pragma once

#include "DeviceBase.hpp"

namespace Device
{
class Gpio : public Device::DeviceBase {
public:
    Gpio(int pinNumber);
    ~Gpio();
private:
    int fd;
};
} // namespace Device


#pragma endregion