#include "GyroScope.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdexcept>
#include <cstring>
#include <cerrno>
#include <vector>

#include <gpiod.h>

#include "utils/logger.hpp"

// ICM-20948 (DS-000189 v1.3) register map
#define ICM20948_REG_BANK_SEL 0x7F
#define ICM20948_WHO_AM_I_VALUE 0xEA
#define ICM20948_BANK_0 0
#define ICM20948_BANK_1 1
#define ICM20948_BANK_2 2
#define ICM20948_BANK_3 3

// Bits/masks used by this driver
#define ICM20948_PWR_MGMT_1_DEVICE_RESET 0x80
#define ICM20948_PWR_MGMT_1_SLEEP 0x40
#define ICM20948_PWR_MGMT_1_CLKSEL_AUTO 0x01

#define ICM20948_INT_PIN_CFG_INT1_LATCH_EN 0x20
#define ICM20948_INT_ENABLE_1_RAW_DATA_0_RDY_EN 0x01

// User bank 0
#define ICM20948_REG_B0_WHO_AM_I 0x00
#define ICM20948_REG_B0_USER_CTRL 0x03
#define ICM20948_REG_B0_LP_CONFIG 0x05
#define ICM20948_REG_B0_PWR_MGMT_1 0x06
#define ICM20948_REG_B0_PWR_MGMT_2 0x07
#define ICM20948_REG_B0_INT_PIN_CFG 0x0F
#define ICM20948_REG_B0_INT_ENABLE 0x10
#define ICM20948_REG_B0_INT_ENABLE_1 0x11
#define ICM20948_REG_B0_INT_ENABLE_2 0x12
#define ICM20948_REG_B0_INT_ENABLE_3 0x13
#define ICM20948_REG_B0_I2C_MST_STATUS 0x17
#define ICM20948_REG_B0_INT_STATUS 0x19
#define ICM20948_REG_B0_INT_STATUS_1 0x1A
#define ICM20948_REG_B0_INT_STATUS_2 0x1B
#define ICM20948_REG_B0_INT_STATUS_3 0x1C
#define ICM20948_REG_B0_DELAY_TIMEH 0x28
#define ICM20948_REG_B0_DELAY_TIMEL 0x29
#define ICM20948_REG_B0_ACCEL_XOUT_H 0x2D
#define ICM20948_REG_B0_ACCEL_XOUT_L 0x2E
#define ICM20948_REG_B0_ACCEL_YOUT_H 0x2F
#define ICM20948_REG_B0_ACCEL_YOUT_L 0x30
#define ICM20948_REG_B0_ACCEL_ZOUT_H 0x31
#define ICM20948_REG_B0_ACCEL_ZOUT_L 0x32
#define ICM20948_REG_B0_GYRO_XOUT_H 0x33
#define ICM20948_REG_B0_GYRO_XOUT_L 0x34
#define ICM20948_REG_B0_GYRO_YOUT_H 0x35
#define ICM20948_REG_B0_GYRO_YOUT_L 0x36
#define ICM20948_REG_B0_GYRO_ZOUT_H 0x37
#define ICM20948_REG_B0_GYRO_ZOUT_L 0x38
#define ICM20948_REG_B0_TEMP_OUT_H 0x39
#define ICM20948_REG_B0_TEMP_OUT_L 0x3A
#define ICM20948_REG_B0_EXT_SLV_SENS_DATA_00 0x3B
#define ICM20948_REG_B0_EXT_SLV_SENS_DATA_01 0x3C
#define ICM20948_REG_B0_EXT_SLV_SENS_DATA_02 0x3D
#define ICM20948_REG_B0_EXT_SLV_SENS_DATA_03 0x3E
#define ICM20948_REG_B0_EXT_SLV_SENS_DATA_04 0x3F
#define ICM20948_REG_B0_EXT_SLV_SENS_DATA_05 0x40
#define ICM20948_REG_B0_EXT_SLV_SENS_DATA_06 0x41
#define ICM20948_REG_B0_EXT_SLV_SENS_DATA_07 0x42
#define ICM20948_REG_B0_EXT_SLV_SENS_DATA_08 0x43
#define ICM20948_REG_B0_EXT_SLV_SENS_DATA_09 0x44
#define ICM20948_REG_B0_EXT_SLV_SENS_DATA_10 0x45
#define ICM20948_REG_B0_EXT_SLV_SENS_DATA_11 0x46
#define ICM20948_REG_B0_EXT_SLV_SENS_DATA_12 0x47
#define ICM20948_REG_B0_EXT_SLV_SENS_DATA_13 0x48
#define ICM20948_REG_B0_EXT_SLV_SENS_DATA_14 0x49
#define ICM20948_REG_B0_EXT_SLV_SENS_DATA_15 0x4A
#define ICM20948_REG_B0_EXT_SLV_SENS_DATA_16 0x4B
#define ICM20948_REG_B0_EXT_SLV_SENS_DATA_17 0x4C
#define ICM20948_REG_B0_EXT_SLV_SENS_DATA_18 0x4D
#define ICM20948_REG_B0_EXT_SLV_SENS_DATA_19 0x4E
#define ICM20948_REG_B0_EXT_SLV_SENS_DATA_20 0x4F
#define ICM20948_REG_B0_EXT_SLV_SENS_DATA_21 0x50
#define ICM20948_REG_B0_EXT_SLV_SENS_DATA_22 0x51
#define ICM20948_REG_B0_EXT_SLV_SENS_DATA_23 0x52
#define ICM20948_REG_B0_FIFO_EN_1 0x66
#define ICM20948_REG_B0_FIFO_EN_2 0x67
#define ICM20948_REG_B0_FIFO_RST 0x68
#define ICM20948_REG_B0_FIFO_MODE 0x69
#define ICM20948_REG_B0_FIFO_COUNTH 0x70
#define ICM20948_REG_B0_FIFO_COUNTL 0x71
#define ICM20948_REG_B0_FIFO_R_W 0x72
#define ICM20948_REG_B0_DATA_RDY_STATUS 0x74
#define ICM20948_REG_B0_FIFO_CFG 0x76
#define ICM20948_REG_B0_REG_BANK_SEL 0x7F

// User bank 1
#define ICM20948_REG_B1_SELF_TEST_X_GYRO 0x02
#define ICM20948_REG_B1_SELF_TEST_Y_GYRO 0x03
#define ICM20948_REG_B1_SELF_TEST_Z_GYRO 0x04
#define ICM20948_REG_B1_SELF_TEST_X_ACCEL 0x0E
#define ICM20948_REG_B1_SELF_TEST_Y_ACCEL 0x0F
#define ICM20948_REG_B1_SELF_TEST_Z_ACCEL 0x10
#define ICM20948_REG_B1_XA_OFFS_H 0x14
#define ICM20948_REG_B1_XA_OFFS_L 0x15
#define ICM20948_REG_B1_YA_OFFS_H 0x17
#define ICM20948_REG_B1_YA_OFFS_L 0x18
#define ICM20948_REG_B1_ZA_OFFS_H 0x1A
#define ICM20948_REG_B1_ZA_OFFS_L 0x1B
#define ICM20948_REG_B1_TIMEBASE_CORRECTION_PLL 0x28
#define ICM20948_REG_B1_REG_BANK_SEL 0x7F

// User bank 2
#define ICM20948_REG_B2_GYRO_SMPLRT_DIV 0x00
#define ICM20948_REG_B2_GYRO_CONFIG_1 0x01
#define ICM20948_REG_B2_GYRO_CONFIG_2 0x02
#define ICM20948_REG_B2_XG_OFFS_USRH 0x03
#define ICM20948_REG_B2_XG_OFFS_USRL 0x04
#define ICM20948_REG_B2_YG_OFFS_USRH 0x05
#define ICM20948_REG_B2_YG_OFFS_USRL 0x06
#define ICM20948_REG_B2_ZG_OFFS_USRH 0x07
#define ICM20948_REG_B2_ZG_OFFS_USRL 0x08
#define ICM20948_REG_B2_ODR_ALIGN_EN 0x09
#define ICM20948_REG_B2_ACCEL_SMPLRT_DIV_1 0x10
#define ICM20948_REG_B2_ACCEL_SMPLRT_DIV_2 0x11
#define ICM20948_REG_B2_ACCEL_INTEL_CTRL 0x12
#define ICM20948_REG_B2_ACCEL_WOM_THR 0x13
#define ICM20948_REG_B2_ACCEL_CONFIG 0x14
#define ICM20948_REG_B2_ACCEL_CONFIG_2 0x15
#define ICM20948_REG_B2_FSYNC_CONFIG 0x52
#define ICM20948_REG_B2_TEMP_CONFIG 0x53
#define ICM20948_REG_B2_MOD_CTRL_USR 0x54
#define ICM20948_REG_B2_REG_BANK_SEL 0x7F

// User bank 3
#define ICM20948_REG_B3_I2C_MST_ODR_CONFIG 0x00
#define ICM20948_REG_B3_I2C_MST_CTRL 0x01
#define ICM20948_REG_B3_I2C_MST_DELAY_CTRL 0x02
#define ICM20948_REG_B3_I2C_SLV0_ADDR 0x03
#define ICM20948_REG_B3_I2C_SLV0_REG 0x04
#define ICM20948_REG_B3_I2C_SLV0_CTRL 0x05
#define ICM20948_REG_B3_I2C_SLV0_DO 0x06
#define ICM20948_REG_B3_I2C_SLV1_ADDR 0x07
#define ICM20948_REG_B3_I2C_SLV1_REG 0x08
#define ICM20948_REG_B3_I2C_SLV1_CTRL 0x09
#define ICM20948_REG_B3_I2C_SLV1_DO 0x0A
#define ICM20948_REG_B3_I2C_SLV2_ADDR 0x0B
#define ICM20948_REG_B3_I2C_SLV2_REG 0x0C
#define ICM20948_REG_B3_I2C_SLV2_CTRL 0x0D
#define ICM20948_REG_B3_I2C_SLV2_DO 0x0E
#define ICM20948_REG_B3_I2C_SLV3_ADDR 0x0F
#define ICM20948_REG_B3_I2C_SLV3_REG 0x10
#define ICM20948_REG_B3_I2C_SLV3_CTRL 0x11
#define ICM20948_REG_B3_I2C_SLV3_DO 0x12
#define ICM20948_REG_B3_I2C_SLV4_ADDR 0x13
#define ICM20948_REG_B3_I2C_SLV4_REG 0x14
#define ICM20948_REG_B3_I2C_SLV4_CTRL 0x15
#define ICM20948_REG_B3_I2C_SLV4_DO 0x16
#define ICM20948_REG_B3_I2C_SLV4_DI 0x17
#define ICM20948_REG_B3_REG_BANK_SEL 0x7F


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

    if (initializeDevice() != 0) {
        if (fd_ != -1) {
            close(fd_);
            fd_ = -1;
        }
        throw std::runtime_error("Failed to initialize ICM-20948");
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
        const int ret = waitInterrupt(timeoutMs);
        if (ret != 0) {
            return ret;
        }
        (void)clearDataReadyInterrupt();
        return 0;
    }

    usleep(timeoutMs * 1000);
    return 0;
}


int GyroScope::getData(int16_t& gx, int16_t& gy, int16_t& gz, int16_t& ax, int16_t& ay, int16_t& az) {
    GyroData data;
    int result = readGyroData(data);
    if (result == 0) {
        gx = data.gx;
        gy = data.gy;
        gz = data.gz;
        ax = data.ax;
        ay = data.ay;
        az = data.az;
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


int GyroScope::writeI2CData(uint8_t reg, const uint8_t* data, size_t length) {
    std::vector<uint8_t> buffer(length + 1);
    buffer[0] = reg;
    std::memcpy(&buffer[1], data, length);
    if (write(fd_, buffer.data(), length + 1) != static_cast<ssize_t>(length + 1)) {
        return -1;
    }
    return 0;
}


int GyroScope::setRegisterBankLocked(uint8_t bank) {
    if (bank > ICM20948_BANK_3) {
        errno = EINVAL;
        return -1;
    }

    if (m_CurrentBank == bank) {
        return 0;
    }

    const uint8_t regVal = static_cast<uint8_t>(bank << 4);
    uint8_t tmp = regVal;
    if (writeI2CData(ICM20948_REG_BANK_SEL, &tmp, 1) != 0) {
        return -1;
    }

    m_CurrentBank = bank;
    return 0;
}


int GyroScope::readRegister(uint8_t bank, uint8_t reg, uint8_t* data, size_t length) {
    std::lock_guard<std::mutex> lock(m_I2cMutex);
    if (setRegisterBankLocked(bank) != 0) {
        return -1;
    }
    return readI2CData(reg, data, length);
}


int GyroScope::writeRegister(uint8_t bank, uint8_t reg, const uint8_t* data, size_t length) {
    std::lock_guard<std::mutex> lock(m_I2cMutex);
    if (setRegisterBankLocked(bank) != 0) {
        return -1;
    }
    return writeI2CData(reg, data, length);
}


int GyroScope::clearDataReadyInterrupt() {
    uint8_t status = 0;
    return readRegister(ICM20948_BANK_0, ICM20948_REG_B0_INT_STATUS_1, &status, 1);
}


int GyroScope::initializeDevice() {
    Logger* logger = Logger::getLoggerInst();

    // Force bank selection on first access.
    m_CurrentBank = 0xFF;

    uint8_t whoami = 0;
    if (readRegister(ICM20948_BANK_0, ICM20948_REG_B0_WHO_AM_I, &whoami, 1) != 0) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to read ICM-20948 WHO_AM_I\r\n");
        return -1;
    }

    if (whoami != ICM20948_WHO_AM_I_VALUE) {
        logger->log(Logger::LOG_LVL_ERROR,
                    "Unexpected WHO_AM_I 0x%02X (expected 0x%02X)\r\n",
                    whoami,
                    ICM20948_WHO_AM_I_VALUE);
        return -1;
    }

    // Reset the device, then wake it up (device comes up in sleep mode).
    const uint8_t reset = ICM20948_PWR_MGMT_1_DEVICE_RESET;
    if (writeRegister(ICM20948_BANK_0, ICM20948_REG_B0_PWR_MGMT_1, &reset, 1) != 0) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to reset ICM-20948\r\n");
        return -1;
    }
    usleep(100000);

    const uint8_t pwrMgmt1 = ICM20948_PWR_MGMT_1_CLKSEL_AUTO;  // SLEEP=0, CLKSEL=auto
    if (writeRegister(ICM20948_BANK_0, ICM20948_REG_B0_PWR_MGMT_1, &pwrMgmt1, 1) != 0) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to wake ICM-20948\r\n");
        return -1;
    }
    usleep(10000);

    const uint8_t pwrMgmt2 = 0x00;  // enable accel+gyro
    if (writeRegister(ICM20948_BANK_0, ICM20948_REG_B0_PWR_MGMT_2, &pwrMgmt2, 1) != 0) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to enable ICM-20948 sensors\r\n");
        return -1;
    }

    // If we're using a GPIO IRQ, configure the INT pin to latch so events aren't missed,
    // and enable RAW_DATA ready interrupt on INT1.
    if (m_IrqRequest) {
        const uint8_t intPinCfg = ICM20948_INT_PIN_CFG_INT1_LATCH_EN;
        if (writeRegister(ICM20948_BANK_0, ICM20948_REG_B0_INT_PIN_CFG, &intPinCfg, 1) != 0) {
            logger->log(Logger::LOG_LVL_ERROR, "Failed to configure ICM-20948 INT pin\r\n");
            return -1;
        }

        const uint8_t intEnable1 = ICM20948_INT_ENABLE_1_RAW_DATA_0_RDY_EN;
        if (writeRegister(ICM20948_BANK_0, ICM20948_REG_B0_INT_ENABLE_1, &intEnable1, 1) != 0) {
            logger->log(Logger::LOG_LVL_ERROR, "Failed to enable ICM-20948 data-ready interrupt\r\n");
            return -1;
        }

        (void)clearDataReadyInterrupt();
    }

    logger->log(Logger::LOG_LVL_INFO, "ICM-20948 initialized (WHO_AM_I=0x%02X)\r\n", whoami);
    return 0;
}


int GyroScope::readGyroData(GyroData& data) {
    uint8_t rawData[6];
    if (readRegister(ICM20948_BANK_0, ICM20948_REG_B0_GYRO_XOUT_H, rawData, 6) != 0) {
        return -1;
    }

    if (readRegister(ICM20948_BANK_0, ICM20948_REG_B0_ACCEL_XOUT_H, rawData, 6) != 0) {
        return -1;
    }
    data.gx = (static_cast<int16_t>(rawData[0]) << 8) | rawData[1];
    data.gy = (static_cast<int16_t>(rawData[2]) << 8) | rawData[3];
    data.gz = (static_cast<int16_t>(rawData[4]) << 8) | rawData[5];
    data.ax = (static_cast<int16_t>(rawData[0]) << 8) | rawData[1];
    data.ay = (static_cast<int16_t>(rawData[2]) << 8) | rawData[3];
    data.az = (static_cast<int16_t>(rawData[4]) << 8) | rawData[5];
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

        if (m_IrqRequest) {
            (void)clearDataReadyInterrupt();
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
