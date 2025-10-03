#include "peripheral_driver.hpp"
#include "types.h"
#include <cerrno>
#include <cstdint>
#include <fcntl.h>
#include <string>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <iostream>
#include <cstring>
#include <stdexcept>

#include "utils/logger.hpp"


#define MAX_ATTEMPT_COUNT   5u


PeripheralCtrl::PeripheralCtrl():
speed(100000),
bitsPerWord(8) {

}


PeripheralCtrl::~PeripheralCtrl() {
    close(this->spiFd);
}


bool PeripheralCtrl::doConfigureDevice(void) {
    if (!this->configSPI()) {
        return false;
    }

    // Add further configuration here

    return true;
}


/**
 * @brief Detect whether the peripheral controller is connected
 * 
 * @return true Controller connected
 * @return false No peripheral controller detected
 */
bool PeripheralCtrl::doDetectDevice(uint32_t& version) {
    // Attempt to read the dummy register
    bool ret;
    uint8_t errCount = 0;
    val_type_t data;
    Logger* logger = Logger::getLoggerInst();
    
    for (int i = 0; i < MAX_ATTEMPT_COUNT; i ++) {
        ret = this->xfer(&data, REG_NOOP);
        if (!ret) {
            continue;
        }
        
        this->isDeviceConnected_ = true;
    }

    ret = this->xfer(&data, REG_VER_MAJOR);
    if (!ret) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to read version major register.\r\n");
        return false;
    }
    this->psocData.version_major.u8 = data.u8;  // Fixed assignment

    ret = this->xfer(&data, REG_VER_MINOR);
    if (!ret) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to read version minor register.\r\n");
        return false;
    }
    this->psocData.version_minor.u8 = data.u8;  // Fixed assignment

    ret = this->xfer(&data, REG_VER_BUILD);
    if (!ret) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to read version build register.\r\n");
        return false;
    }
    this->psocData.version_build.u8 = data.u8;  // Fixed assignment
    logger->log(Logger::LOG_LVL_INFO,  "Peripheral controller detected:\r\nVersion: %u.%u.%u\r\n",static_cast<int>(this->psocData.version_major.u8),
                                                                                                  static_cast<int>(this->psocData.version_minor.u8),
                                                                                                  static_cast<int>(this->psocData.version_build.u8));

    version = this->psocData.version_major.u8 << 16 | this->psocData.version_major.u8 << 8 | this->psocData.version_major.u8;
    return true;
}


/**
 * @brief Set the drive mode of the peripheral controller
 * 
 * @param state true for forward, false for reverse
 * @return int 0 on success, -1 on failure
 */
int PeripheralCtrl::setMotorState(bool state) {
    bool ret;
    val_type_t data;

    data.u8 = state;
    ret = this->xfer(&data, PeripheralCtrl::REG_SET_MOTOR_STATUS);
    if (!ret) {
        return -1;
    }

    return 0;
}


/**
 * @brief Set the drive mode of the peripheral controller
 * 
 * @param state true for automatic, false for manual
 * @return int 0 on success, -1 on failure
 */
int PeripheralCtrl::setDriveMode(bool state) {
    bool ret;
    val_type_t data;

    data.u8 = state ? 1 : 0;  // Assuming 1 for forward, 0 for reverse
    ret = this->xfer(&data, PeripheralCtrl::REG_SPEED_SETPOINT);
    if (!ret) {
        return -1;
    }

    return 0;
}


/**
 * @brief Set the PID parameters for the peripheral controller
 * 
 * @param p Proportional gain
 * @param i Integral gain
 * @param d Derivative gain
 * @return int 0 on success, -1 on failure
 */
int PeripheralCtrl::setPIParams(float p, float i, float d) {
    bool ret;
    val_type_t data;

    data.f32 = p;
    ret = this->xfer(&data, PeripheralCtrl::REG_PID_P);
    if (!ret) {
        return -1;
    }

    data.f32 = i;
    ret = this->xfer(&data, PeripheralCtrl::REG_PID_I);
    if (!ret) {
        return -1;
    }

    data.f32 = d;
    ret = this->xfer(&data, PeripheralCtrl::REG_PID_D);
    if (!ret) {
        return -1;
    }

    return 0;
}


/**
 * @brief Read data from the peripheral controller
 * 
 * @param psocData Reference to the psocDataStruct to fill with data
 * @return int 0 on success, -1 on failure
 */
int PeripheralCtrl::readData(psocDataStruct& data) {
    Logger* logger = Logger::getLoggerInst();

    if (!this->isDeviceConnected_) {
        logger->log(Logger::LOG_LVL_ERROR, "Peripheral controller not connected.\r\n");
        return -1;
    }
    
    bool ret = this->xfer(&data.speed, REG_SPEED);
    if (!ret) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to read speed register.\r\n");
        return -1;
    }

    ret = this->xfer(&data.frontDistance, REG_FRONT_DISTANCE);
    if (!ret) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to read front distance register.\r\n");
        return -1;
    }

    ret = this->xfer(&data.leftDistance, REG_LEFT_DISTANCE);
    if (!ret) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to read left distance register.\r\n");
        return -1;
    }

    ret = this->xfer(&data.rightDistance, REG_RIGHT_DISTANCE);
    if (!ret) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to read right distance register.\r\n");
        return -1;
    }

    data.version_major = this->psocData.version_major;
    data.version_minor = this->psocData.version_minor;
    data.version_build = this->psocData.version_build;
    return 0;
}


/**
 * @brief Configure the SPI device
 * 
 * @return true Configuration successful
 * @return false Configuration failed
 */
bool PeripheralCtrl::configSPI(void) {
    std::string spiDev("/dev/spidev0.0");  // <-- include .0 for CS0
    Logger* logger = Logger::getLoggerInst();
    this->spiFd = open(spiDev.c_str(), O_RDWR);
    if (this->spiFd < 0) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to open SPI device: %d\r\n", strerror(errno));
        return false;
    }

    const uint8_t mode = SPI_MODE_0; // Clock idle low, sample on rising edge
    const uint32_t _speed = speed;  // 1 MHz, you can change later
    const uint8_t bitsPerWord = 8;   // 8 bits
    
    // Set SPI mode
    if (ioctl(this->spiFd, SPI_IOC_WR_MODE, &mode) < 0) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to set SPI mode: %d\r\n", strerror(errno));
        close(this->spiFd);
        return false;
    }

    // Set SPI speed
    if (ioctl(this->spiFd, SPI_IOC_WR_MAX_SPEED_HZ, &_speed) < 0) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to set SPI speed: %d\r\n", strerror(errno));
        close(this->spiFd);
        return false;
    }

    // Set bits per word
    if (ioctl(this->spiFd, SPI_IOC_WR_BITS_PER_WORD, &bitsPerWord) < 0) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to set SPI bits per word: %d\r\n", strerror(errno));
        close(this->spiFd);
        return false;
    }

    uint32_t actual_speed;
    if (ioctl(this->spiFd, SPI_IOC_RD_MAX_SPEED_HZ, &actual_speed) == 0) {
        logger->log(Logger::LOG_LVL_INFO, "SPI actual speed set to: %luHz\r\n", actual_speed);
    } else {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to read back SPI speed\r\n");
    }

    logger->log(Logger::LOG_LVL_INFO, "SPI device configured successfully\r\n");
    return true;
}


/**
 * @brief Transfer data to the peripheral controller
 * 
 * @param data Pointer to the data structure to transfer
 * @return true Transfer successful
 * @return false Transfer failed
 */
bool PeripheralCtrl::xfer(val_type_t* data, uint8_t reg) {
    if (data == nullptr) {
        return false;
    }

    bool ret;
    PeripheralCtrl::spiTransactionStruct dataOut;
    dataOut.ack = 0x00;
    dataOut.transactionType = 0xFF;
    dataOut.reg = reg;
    dataOut.data.u32 = data->u32;

    ret = this->xferSPI(reinterpret_cast<uint8_t*>(&dataOut), sizeof(PeripheralCtrl::spiTransactionStruct));
    if ( !ret || !dataOut.ack ) {
        return false;
    }

    data->u32 = dataOut.data.u32;
    return true;
}


/**
 * @brief Transfer via the spi file write
 * 
 * @param pbuf Pointer to the transmissio buffer
 * @param length Length of data to send
 * @return true 
 * @return false 
 */
bool PeripheralCtrl::xferSPI(uint8_t* pbuf, size_t length) {
    if (pbuf == nullptr || length == 0) {
        return false;
    }

    uint8_t rxbuf[sizeof(PeripheralCtrl::spiTransactionStruct)] = {0};  // receive buffer
    Logger* logger = Logger::getLoggerInst();
    
    struct spi_ioc_transfer tr = {};
    tr.tx_buf = reinterpret_cast<unsigned long>(pbuf);
    tr.rx_buf = reinterpret_cast<unsigned long>(rxbuf);
    tr.len = static_cast<uint32_t>(length);
    tr.delay_usecs = 0;
    tr.speed_hz = this->speed;
    tr.bits_per_word = this->bitsPerWord;

    int ret = ioctl(this->spiFd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to perform SPI transfer: %d\r\n", strerror(errno));
        return false;
    }

    memcpy(pbuf, rxbuf, length);
    return true;
}
