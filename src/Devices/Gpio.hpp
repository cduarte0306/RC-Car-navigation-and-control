#pragma once

#include <unordered_map>
#include "DeviceBase.hpp"

struct gpiod_chip;
struct gpiod_line_request;

namespace Device
{
class Gpio : public Device::DeviceBase {
public:
    Gpio(int lineOffset, int chipIndex = 0);
    ~Gpio();

    static Gpio* create(int pinNumber);

    /** 
     * @brief Write a value to the GPIO pin
     * 
     * @param value Value to write (0 or 1)
     * @return 
     * 
     */
    int gpioWrite(int value);
private:
    struct HeaderGpio
    {
        int header_pin;        // Physical header pin number (1–40)
        const char* soc_name;  // SoC pad name (for debugging)
        int gpiochip;          // gpiochip index (0 or 1)
        int line_offset;       // line offset within gpiochip
    };

    static const std::unordered_map<int, HeaderGpio> jetson_orin_nano_gpio_map_lookup;

    gpiod_chip* m_chip{nullptr};
    gpiod_line_request* m_request{nullptr};
    unsigned int m_lineOffset{0};
    int fd{-1};
};
} // namespace Device


#pragma endregion