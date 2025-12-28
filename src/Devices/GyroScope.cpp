#include "GyroScope.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdexcept>
#include <cstring>
#include <cerrno>

#include <gpiod.h>

#include "utils/logger.hpp"


namespace Device {
GyroScope::GyroScope(const char* device, const char* irqChip, int irqLineOffset)
    : devicePath_(device) {
    if (initializeI2C() != 0) {
        throw std::runtime_error("Failed to initialize I2C communication");
    }

    if (irqChip && irqLineOffset >= 0) {
        Logger* logger = Logger::getLoggerInst();
        const int ret = initializeInterrupt(irqChip, static_cast<unsigned int>(irqLineOffset));
        if (ret != 0) {
            logger->log(Logger::LOG_LVL_ERROR,
                        "Failed to initialize gyro IRQ (%s:%d), falling back to timed polling\r\n",
                        irqChip, irqLineOffset);
        }
    }

    // Run poll thread
    m_PollThread = std::thread(&GyroScope::pollGyroData, this);
}


GyroScope::~GyroScope() {
    m_ThreadCanRun = false;

    if (m_PollThread.joinable()) {
        m_PollThread.join();
    }

    if (fd_ != -1) {
        close(fd_);
    }

    if (m_IrqRequest) {
        gpiod_line_request_release(m_IrqRequest);
        m_IrqRequest = nullptr;
    }

    if (m_IrqEventBuffer) {
        gpiod_edge_event_buffer_free(m_IrqEventBuffer);
        m_IrqEventBuffer = nullptr;
    }

    if (m_IrqChip) {
        gpiod_chip_close(m_IrqChip);
        m_IrqChip = nullptr;
    }
}


int GyroScope::waitFrameSynchReady(int timeoutMs) {
    // If an interrupt line is configured, wait on its edge; otherwise fall back
    // to the old sleep-based placeholder.
    if (m_IrqRequest) {
        return waitInterrupt(timeoutMs);
    }

    usleep(timeoutMs * 1000);
    return 0;
}


int GyroScope::getData(int16_t& gx, int16_t& gy, int16_t& gz) {
    GyroData data;
    int result = readGyroData(data);
    if (result == 0) {
        gx = data.gx;
        gy = data.gy;
        gz = data.gz;
    }
    return result;
}


int GyroScope::initializeI2C() {
    Logger* logger = Logger::getLoggerInst();
    fd_ = open(devicePath_, O_RDWR);
    if (fd_ < 0) {
        throw std::runtime_error("Failed to open I2C bus");
    }

    if (ioctl(fd_, I2C_SLAVE, gyroAddress_) < 0) {
        close(fd_);
        throw std::runtime_error("Failed to acquire bus access and/or talk to slave");
    }
    logger->log(Logger::LOG_LVL_INFO, "I2C communication initialized on %s\r\n", devicePath_);
    return 0;
}


int GyroScope::readI2CData(uint8_t reg, uint8_t* data, size_t length) {
    if (write(fd_, &reg, 1) != 1) {
        return -1;
    }
    if (read(fd_, data, length) != static_cast<ssize_t>(length)) {
        return -1;
    }
    return 0;
}


int GyroScope::writeI2CData(uint8_t reg, uint8_t* data, size_t length) {
    uint8_t buffer[length + 1];
    buffer[0] = reg;
    std::memcpy(&buffer[1], data, length);
    if (write(fd_, buffer, length + 1) != static_cast<ssize_t>(length + 1)) {
        return -1;
    }
    return 0;
}


int GyroScope::readGyroData(GyroData& data) {
    uint8_t rawData[6];
    if (readI2CData(0x43, rawData, 6) != 0) {
        return -1;
    }
    data.gx = (static_cast<int16_t>(rawData[0]) << 8) | rawData[1];
    data.gy = (static_cast<int16_t>(rawData[2]) << 8) | rawData[3];
    data.gz = (static_cast<int16_t>(rawData[4]) << 8) | rawData[5];
    return 0;
}


void GyroScope::pollGyroData(void) {
    // If an interrupt line is configured, use it to block until data-ready.
    // Otherwise fall back to periodic reads.
    while (m_ThreadCanRun.load()) {
        if (m_IrqRequest) {
            // Use a finite timeout so the thread can exit promptly.
            const int ret = waitInterrupt(250);
            if (ret != 0) {
                continue;
            }
        } else {
            usleep(10000);  // Poll every 10 ms
        }

        GyroData data;
        if (readGyroData(data) == 0) {
            m_GyroBuffer.push(data);
        }
    }
}

int GyroScope::initializeInterrupt(const char* irqChip, unsigned int irqLineOffset) {
    Logger* logger = Logger::getLoggerInst();

    m_IrqChip = gpiod_chip_open(irqChip);
    if (!m_IrqChip) {
        logger->log(Logger::LOG_LVL_ERROR, "gpiod_chip_open(%s) failed: %s\r\n", irqChip, strerror(errno));
        return -1;
    }

    struct gpiod_line_settings* settings = gpiod_line_settings_new();
    if (!settings) {
        logger->log(Logger::LOG_LVL_ERROR, "gpiod_line_settings_new failed: %s\r\n", strerror(errno));
        return -1;
    }

    struct gpiod_line_config* lineCfg = gpiod_line_config_new();
    if (!lineCfg) {
        gpiod_line_settings_free(settings);
        logger->log(Logger::LOG_LVL_ERROR, "gpiod_line_config_new failed: %s\r\n", strerror(errno));
        return -1;
    }

    struct gpiod_request_config* reqCfg = gpiod_request_config_new();
    if (!reqCfg) {
        gpiod_line_config_free(lineCfg);
        gpiod_line_settings_free(settings);
        logger->log(Logger::LOG_LVL_ERROR, "gpiod_request_config_new failed: %s\r\n", strerror(errno));
        return -1;
    }

    // Configure as input + rising edge detection.
    if (gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT) != 0) {
        logger->log(Logger::LOG_LVL_ERROR, "gpiod_line_settings_set_direction failed: %s\r\n", strerror(errno));
        gpiod_request_config_free(reqCfg);
        gpiod_line_config_free(lineCfg);
        gpiod_line_settings_free(settings);
        return -1;
    }

    if (gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_RISING) != 0) {
        logger->log(Logger::LOG_LVL_ERROR, "gpiod_line_settings_set_edge_detection failed: %s\r\n", strerror(errno));
        gpiod_request_config_free(reqCfg);
        gpiod_line_config_free(lineCfg);
        gpiod_line_settings_free(settings);
        return -1;
    }

    const unsigned int offsets[] = {irqLineOffset};
    if (gpiod_line_config_add_line_settings(lineCfg, offsets, 1, settings) != 0) {
        logger->log(Logger::LOG_LVL_ERROR, "gpiod_line_config_add_line_settings failed: %s\r\n", strerror(errno));
        gpiod_request_config_free(reqCfg);
        gpiod_line_config_free(lineCfg);
        gpiod_line_settings_free(settings);
        return -1;
    }

    gpiod_request_config_set_consumer(reqCfg, "rc-car-nav-gyro");

    m_IrqRequest = gpiod_chip_request_lines(m_IrqChip, reqCfg, lineCfg);
    if (!m_IrqRequest) {
        logger->log(Logger::LOG_LVL_ERROR, "gpiod_chip_request_lines failed: %s\r\n", strerror(errno));
        gpiod_request_config_free(reqCfg);
        gpiod_line_config_free(lineCfg);
        gpiod_line_settings_free(settings);
        return -1;
    }

    m_IrqEventBuffer = gpiod_edge_event_buffer_new(1);
    if (!m_IrqEventBuffer) {
        logger->log(Logger::LOG_LVL_ERROR, "gpiod_edge_event_buffer_new failed: %s\r\n", strerror(errno));
        return -1;
    }

    // These objects are no longer needed after creating the request.
    gpiod_request_config_free(reqCfg);
    gpiod_line_config_free(lineCfg);
    gpiod_line_settings_free(settings);

    logger->log(Logger::LOG_LVL_INFO, "Gyro IRQ configured on %s:%u\r\n", irqChip, irqLineOffset);
    return 0;
}

int GyroScope::waitInterrupt(int timeoutMs) {
    if (!m_IrqRequest) {
        errno = ENODEV;
        return -1;
    }

    const int64_t timeoutNs = (timeoutMs < 0) ? -1 : static_cast<int64_t>(timeoutMs) * 1000000LL;
    const int ret = gpiod_line_request_wait_edge_events(m_IrqRequest, timeoutNs);
    if (ret <= 0) {
        if (ret == 0) {
            errno = ETIMEDOUT;
        }
        return -1;
    }

    // Drain at least one event.
    const int readRet = gpiod_line_request_read_edge_events(m_IrqRequest, m_IrqEventBuffer, 1);
    if (readRet < 0) {
        return -1;
    }

    return 0;
}
}  // namespace Device