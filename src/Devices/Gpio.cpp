#include "Gpio.hpp"
#include <fcntl.h>

#include <cstdio>

#include <gpiod.h>


namespace Device
{

const std::unordered_map<int, Gpio::HeaderGpio> Gpio::jetson_orin_nano_gpio_map_lookup = {
    {7,  {7,  "PAC.06",  0, 144 }},   // GPIO09 (you remapped this)
    {11, {11, "PH.04",   0, 47  }},   // GPIO17
    {13, {13, "PH.05",   0, 48  }},   // GPIO27
    {15, {15, "PN.01",   0, 85  }},   // GPIO22 (can be PWM)
    {29, {29, "PQ.05",   0, 105 }},   // GPIO05
    {31, {31, "PQ.06",   0, 106 }},   // GPIO06
    {32, {32, "PG.06",   0, 41  }},   // GPIO12 (PWM7)
    {33, {33, "PH.00",   0, 43  }},   // GPIO13 (PWM5)
    {35, {35, "PI.02",   0, 53  }},   // GPIO19
    {37, {37, "PY.02",   0, 124 }},   // GPIO26
};

Gpio::Gpio(int lineOffset, int chipIndex) : Device::DeviceBase(), m_lineOffset(static_cast<unsigned int>(lineOffset)) {
    char chip_name[20];
    snprintf(chip_name, sizeof(chip_name), "/dev/gpiochip%d", chipIndex);

    m_chip = gpiod_chip_open(chip_name);
    if (!m_chip) {
        return;
    }

    gpiod_line_settings* settings = gpiod_line_settings_new();
    if (!settings) {
        gpiod_chip_close(m_chip);
        m_chip = nullptr;
        return;
    }

    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
    gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);

    gpiod_line_config* config = gpiod_line_config_new();
    if (!config) {
        gpiod_line_settings_free(settings);
        gpiod_chip_close(m_chip);
        m_chip = nullptr;
        return;
    }

    if (gpiod_line_config_add_line_settings(config, &m_lineOffset, 1, settings) < 0) {
        gpiod_line_config_free(config);
        gpiod_line_settings_free(settings);
        gpiod_chip_close(m_chip);
        m_chip = nullptr;
        return;
    }

    gpiod_request_config* reqConfig = gpiod_request_config_new();
    if (!reqConfig) {
        gpiod_line_config_free(config);
        gpiod_line_settings_free(settings);
        gpiod_chip_close(m_chip);
        m_chip = nullptr;
        return;
    }

    gpiod_request_config_set_consumer(reqConfig, "rc_car_gpio");

    m_request = gpiod_chip_request_lines(m_chip, reqConfig, config);
    gpiod_request_config_free(reqConfig);
    gpiod_line_config_free(config);
    gpiod_line_settings_free(settings);

    if (!m_request) {
        gpiod_chip_close(m_chip);
        m_chip = nullptr;
        return;
    }

    fd = gpiod_line_request_get_fd(m_request);
}


Gpio::~Gpio() {
    if (m_request) {
        gpiod_line_request_release(m_request);
        m_request = nullptr;
    }

    if (m_chip) {
        gpiod_chip_close(m_chip);
        m_chip = nullptr;
    }
}


Gpio* Gpio::create(int pinNumber) {
    auto it = jetson_orin_nano_gpio_map_lookup.find(pinNumber);
    if (it == jetson_orin_nano_gpio_map_lookup.end()) {
        return nullptr; // Invalid pin number
    }
    const HeaderGpio& gpioInfo = it->second;
    return new Gpio(gpioInfo.line_offset, gpioInfo.gpiochip);
}


int Gpio::gpioWrite(int value) {
    if (!m_request) {
        return -1; // Line request not initialized
    }

    int rc = gpiod_line_request_set_value(
        m_request,
        m_lineOffset,
        value ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);

    return rc;
}
} // namespace Device