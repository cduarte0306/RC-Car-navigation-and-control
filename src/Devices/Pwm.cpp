#pragma once

#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#include <errno.h>
#include <stdexcept>

#include "DeviceBase.hpp"
#include "Pwm.hpp"

/* Must match kernel defines */
#define TEGRA_PWM_IOC_MAGIC      'P'
#define TEGRA_PWM_IOC_GET_DUTY   _IOR(TEGRA_PWM_IOC_MAGIC, 0, unsigned int)
#define TEGRA_PWM_IOC_SET_DUTY   _IOW(TEGRA_PWM_IOC_MAGIC, 1, unsigned int)
#define TEGRA_PWM_IOC_GET_FREQ   _IOR(TEGRA_PWM_IOC_MAGIC, 2, unsigned int)
#define TEGRA_PWM_IOC_SET_FREQ   _IOW(TEGRA_PWM_IOC_MAGIC, 3, unsigned int)
#define TEGRA_PWM_IOC_ENABLE     _IO(TEGRA_PWM_IOC_MAGIC, 4)
#define TEGRA_PWM_IOC_DISABLE    _IO(TEGRA_PWM_IOC_MAGIC, 5)


namespace Device
{  
Pwm::Pwm(const char* devName, int freq) : DeviceBase() {
    const char *dev = devName;

    this->fd = open(dev, O_RDWR);
    if (fd < 0) {
        throw(std::runtime_error("Failed to open " + (std::string(devName))));
    }

    int ret = doIoctl(this->fd, TEGRA_PWM_IOC_SET_FREQ, reinterpret_cast<uint32_t*>(&freq));
    if (ret < 0) {
        throw(std::runtime_error("Value cannot be negative."));
    }
}

Pwm::~Pwm() {
    close(this->fd);
}

/**
 * @brief Enable/Disable PWM
 * 
 * @param status True: Enable
 *               False: Disable
 * @return int 
 */
int Pwm::writeEnable(bool status) {
    if (this->fd < 0) {
        return -1;
    }
    return doIoctl(this->fd, (status) ? (TEGRA_PWM_IOC_ENABLE) : (TEGRA_PWM_IOC_DISABLE), NULL);
}


/**
 * @brief Write the PWM duty cycle
 * 
 * @param period PWM period
 * @return int 
 */
int Pwm::writeDutyCycle(int period) {
    if (this->fd < 0) {
        return -1;
    }
    return doIoctl(this->fd, TEGRA_PWM_IOC_SET_DUTY, &period);
}

int Pwm::doIoctl(int fd, unsigned long cmd, void *arg) {
    int ret = ioctl(fd, cmd, arg);
    if (ret < 0)
        fprintf(stderr, "ioctl 0x%lx failed: %s\n", cmd, strerror(errno));
    return ret;
}
};    

#pragma endregion