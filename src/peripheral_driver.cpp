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
#include <termios.h>
#include <sstream>
#include <chrono>
#include <thread>


#define MAX_ATTEMPT_COUNT   5u


PeripheralCtrl::PeripheralCtrl():
speed(100000),
bitsPerWord(8) {
}


PeripheralCtrl::~PeripheralCtrl() {
    close(this->spiFd);
}


/**
 * 
 */
bool PeripheralCtrl::doConfigureDevice(void) {
    if (!this->configSerial()) {
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
    
    for (int i = 0; i < MAX_ATTEMPT_COUNT; i ++) {
#if defined(__aarch64__)
        ret = this->xfer(&data, REG_NOOP);
#else
        ret = this->rdReg(data, REG_NOOP);
#endif
        if (!ret) {
            continue;
        }
        
        this->isDeviceConnected_ = true;
    }

#if defined(__aarch64__)
    ret = this->xfer(&data, REG_VER_MAJOR);
    if (!ret) {
        std::cerr << "Failed to read version major register." << std::endl;
        return false;
    }
    this->psocData.version_major.u8 = data.u8;  // Fixed assignment

    ret = this->xfer(&data, REG_VER_MINOR);
    if (!ret) {
        std::cerr << "Failed to read version minor register." << std::endl;
        return false;
    }
    this->psocData.version_minor.u8 = data.u8;  // Fixed assignment

    ret = this->xfer(&data, REG_VER_BUILD);
    if (!ret) {
        std::cerr << "Failed to read version build register." << std::endl;
        return false;
    }
    this->psocData.version_build.u8 = data.u8;  // Fixed assignment

#else
    ret = this->rdReg(this->psocData.version_major, REG_VER_MAJOR);
    if (!ret) {
        std::cerr << "Failed to read version major register." << std::endl;
        return false;
    }

    ret = this->rdReg(this->psocData.version_minor, REG_VER_MINOR);
    if (!ret) {
        std::cerr << "Failed to read version minor register." << std::endl;
        return false;
    }

    ret = this->rdReg(this->psocData.version_build, REG_VER_BUILD);
    if (!ret) {
        std::cerr << "Failed to read version build register." << std::endl;
        return false;
    }
#endif

    std::cout << "Peripheral controller detected: "
              << "Version " << static_cast<int>(this->psocData.version_major.u8) << "."  // Fixed static_cast
              << static_cast<int>(this->psocData.version_minor.u8) << "."  // Fixed static_cast
              << static_cast<int>(this->psocData.version_build.u8) << std::endl;  // Fixed static_cast

    version = this->psocData.version_major.u8 << 16 | this->psocData.version_minor.u8 << 8 | this->psocData.version_build.u8;
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
    if (!this->isDeviceConnected_) {
        std::cerr << "Peripheral controller not connected." << std::endl;
        return -1;
    }
    bool ret;

#if defined(__aarch64__)
    ret = this->xfer(&data.speed, REG_SPEED);
    if (!ret) {
        std::cerr << "Failed to read speed register." << std::endl;
        return -1;
    }

    ret = this->xfer(&data.frontDistance, REG_FRONT_DISTANCE);
    if (!ret) {
        std::cerr << "Failed to read front distance register." << std::endl;
        return -1;
    }

    ret = this->xfer(&data.leftDistance, REG_LEFT_DISTANCE);
    if (!ret) {
        std::cerr << "Failed to read left distance register." << std::endl;
        return -1;
    }

    ret = this->xfer(&data.rightDistance, REG_RIGHT_DISTANCE);
    if (!ret) {
        std::cerr << "Failed to read right distance register." << std::endl;
        return -1;
    }
#else
    ret = this->rdReg(data.speed, REG_SPEED);
    if (!ret) {
        std::cerr << "Failed to read speed register." << std::endl;
        return -1;
    }

    ret = this->rdReg(data.frontDistance, REG_FRONT_DISTANCE);
    if (!ret) {
        std::cerr << "Failed to read front distance register." << std::endl;
        return -1;
    }

    ret = this->rdReg(data.leftDistance, REG_LEFT_DISTANCE);
    if (!ret) {
        std::cerr << "Failed to read left distance register." << std::endl;
        return -1;
    }

    ret = this->rdReg(data.rightDistance, REG_RIGHT_DISTANCE);
    if (!ret) {
        std::cerr << "Failed to read right distance register." << std::endl;
        return -1;
    }
#endif

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
bool PeripheralCtrl::configSerial(void) {
#if defined(__aarch64__)
    std::string spiDev("/dev/spidev0.0");  // <-- include .0 for CS0
    this->spiFd = open(spiDev.c_str(), O_RDWR);
    if (this->spiFd < 0) {
        std::cerr << "Failed to open SPI device: " << strerror(errno) << std::endl;
        return false;
    }

    const uint8_t mode = SPI_MODE_0; // Clock idle low, sample on rising edge
    const uint32_t _speed = speed;  // 1 MHz, you can change later
    const uint8_t bitsPerWord = 8;   // 8 bits
    
    // Set SPI mode
    if (ioctl(this->spiFd, SPI_IOC_WR_MODE, &mode) < 0) {
        std::cerr << "Failed to set SPI mode: " << strerror(errno) << std::endl;
        close(this->spiFd);
        return false;
    }

    // Set SPI speed
    if (ioctl(this->spiFd, SPI_IOC_WR_MAX_SPEED_HZ, &_speed) < 0) {
        std::cerr << "Failed to set SPI speed: " << strerror(errno) << std::endl;
        close(this->spiFd);
        return false;
    }

    // Set bits per word
    if (ioctl(this->spiFd, SPI_IOC_WR_BITS_PER_WORD, &bitsPerWord) < 0) {
        std::cerr << "Failed to set SPI bits per word: " << strerror(errno) << std::endl;
        close(this->spiFd);
        return false;
    }

    uint32_t actual_speed;
    if (ioctl(this->spiFd, SPI_IOC_RD_MAX_SPEED_HZ, &actual_speed) == 0) {
        std::cout << "SPI actual speed set to: " << actual_speed << " Hz" << std::endl;
    } else {
        std::cerr << "Failed to read back SPI speed." << std::endl;
    }

    std::cout << "SPI device configured successfully." << std::endl;
    return true;

#else
    this->serialFd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY); 
    if (this->serialFd == -1) {
        perror("open");
        return 1;
    }

    struct termios options;
    tcgetattr(this->serialFd, &options);
    cfsetispeed(&options, B115200);  // set baud rate
    cfsetospeed(&options, B115200);
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input mode
    options.c_oflag &= ~OPOST; // Raw output
    options.c_cc[VMIN] = 1;  // Read at least 1 byte
    options.c_cc[VTIME] = 1; // 100ms timeout

    tcsetattr(this->serialFd, TCSANOW, &options);
#endif

    std::cout << "Serial device configured successfully." << std::endl;
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
 * @brief Write data to the peripheral controller via serial
 * 
 * @param data Data to write
 * @param reg Register to write to
 * @return true Write successful
 * @return false Write failed
 */
bool PeripheralCtrl::wrtReg(val_type_t data, uint8_t reg) {
    if (data.u32 == 0) {
        return false;
    }

    std::stringstream ss;
    ss << "write-cmd-reg " << static_cast<int>(reg) << " " << data.u32 << "\n";
    ssize_t written = write(this->serialFd, ss.str().c_str(), ss.str().size());
    if (written < 0) {
        std::cerr << "Failed to write to serial: " << strerror(errno) << std::endl;
        return false;
    }

    return true;
}


/**
 * @brief Read data from the peripheral controller via serial
 * 
 * @param data Reference to the data structure to fill with read data
 * @param reg Register to read from
 * @return true Read successful
 * @return false Read failed
 */
bool PeripheralCtrl::rdReg(val_type_t& data, uint8_t reg) {
    int n = 0;
    tcflush(this->serialFd, TCIOFLUSH);  // Clear both in and out
    std::stringstream ss;
    ss << "read-cmd-reg " << static_cast<int>(reg) << "\r\n";
    std::string cmd = ss.str();

    for (char c : cmd) {
        write(this->serialFd, &c, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    usleep(100000);  // Give time for MCU to respond

    memset(this->serialBuffer, 0, sizeof(this->serialBuffer));
    n = read(this->serialFd, this->serialBuffer, sizeof(this->serialBuffer));
    if (n < 0) {
        std::cerr << "Failed to read from serial: " << strerror(errno) << std::endl;
        return false;
    }

    std::string response(this->serialBuffer, n);

    auto pos = response.find("Reg value: ");
    if (pos == std::string::npos) {
        std::cerr << "Invalid response from peripheral controller: " << response << std::endl;
        return false;
    }

    std::string valueStr = response.substr(pos + std::string("Reg value: ").length());
    try {
        data.u32 = std::stoul(valueStr);
    } catch (const std::invalid_argument& e) {
        std::cerr << "Invalid data received: " << valueStr << std::endl;
        return false;
    } catch (const std::out_of_range& e) {
        std::cerr << "Data out of range: " << valueStr << std::endl;
        return false;
    }

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

    struct spi_ioc_transfer tr = {};
    tr.tx_buf = reinterpret_cast<unsigned long>(pbuf);
    tr.rx_buf = reinterpret_cast<unsigned long>(rxbuf);
    tr.len = static_cast<uint32_t>(length);
    tr.delay_usecs = 0;
    tr.speed_hz = this->speed;
    tr.bits_per_word = this->bitsPerWord;

    int ret = ioctl(this->spiFd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1) {
        std::cerr << "Failed to perform SPI transfer: " << strerror(errno) << std::endl;
        return false;
    }

    memcpy(pbuf, rxbuf, length);
    return true;
}
