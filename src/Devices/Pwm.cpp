#pragma once

#include <stdio.h>
#include <stdint.h>
#include <algorithm>
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
#define TEGRA_PWM_IOC_GET_DUTY   _IOR(TEGRA_PWM_IOC_MAGIC, 0, uint32_t)
#define TEGRA_PWM_IOC_SET_DUTY   _IOW(TEGRA_PWM_IOC_MAGIC, 1, uint32_t)
#define TEGRA_PWM_IOC_GET_FREQ   _IOR(TEGRA_PWM_IOC_MAGIC, 2, uint32_t)
#define TEGRA_PWM_IOC_SET_FREQ   _IOW(TEGRA_PWM_IOC_MAGIC, 3, uint32_t)
#define TEGRA_PWM_IOC_ENABLE     _IO(TEGRA_PWM_IOC_MAGIC, 4)
#define TEGRA_PWM_IOC_DISABLE    _IO(TEGRA_PWM_IOC_MAGIC, 5)


namespace Device
{  
static uint32_t clampDutyPercent(int dutyPercent) {
    return static_cast<uint32_t>(std::clamp(dutyPercent, 0, 100));
}

Pwm::Pwm(const char* devName, int freq) : DeviceBase() {
    const char *dev = devName;

    this->fd = open(dev, O_RDWR);
    if (fd < 0) {
        throw(std::runtime_error("Failed to open " + (std::string(devName))));
    }

    if (freq <= 0) {
        close(this->fd);
        this->fd = -1;
        throw(std::runtime_error("PWM frequency must be > 0"));
    }

    uint32_t freq_u32 = static_cast<uint32_t>(freq);
    int ret = doIoctl(this->fd, TEGRA_PWM_IOC_SET_FREQ, &freq_u32);
    if (ret < 0) {
        close(this->fd);
        this->fd = -1;
        throw(std::runtime_error("Failed to set PWM frequency"));
    }
}

Pwm::~Pwm() {
    (void)writeEnable(false);
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
    return doIoctl(this->fd, (status) ? (TEGRA_PWM_IOC_ENABLE) : (TEGRA_PWM_IOC_DISABLE), NULL);
}


/**
 * @brief Write the PWM duty cycle
 * 
 * @param period PWM period
 * @return int 
 */
int Pwm::writeDutyCycle(int dutyPercent) {
    uint32_t duty_u32 = clampDutyPercent(dutyPercent);
    return doIoctl(fd, TEGRA_PWM_IOC_SET_DUTY, &duty_u32);
}


int Pwm::doIoctl(int fd, unsigned long cmd, void *arg) {
    int ret = ioctl(fd, cmd, arg);
    if (ret < 0)
        fprintf(stderr, "ioctl 0x%lx failed: %s\n", cmd, strerror(errno));
    return ret;
}
};    

#pragma endregion
