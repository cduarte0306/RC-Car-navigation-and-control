#ifndef STEREOCAMGYRO_CPP
#define STEREOCAMGYRO_CPP

#include <cstddef>
#include <stdint.h>
#include <atomic>
#include <mutex>
#include <thread>
#include <RcMessageLib.hpp>
#include "DeviceBase.hpp"


struct gpiod_chip;
struct gpiod_line_request;
struct gpiod_edge_event_buffer;

namespace Device {
class GyroScope : public Device::DeviceBase {
public:
    GyroScope(const char* device = "/dev/i2c-0",
                  const char* irqChip = nullptr,
                  int irqLineOffset = -1);

    GyroScope(const GyroScope&) = delete;

    GyroScope& operator=(const GyroScope&) = delete;

    ~GyroScope();

    struct GyroData {
        int16_t gx;
        int16_t gy;
        int16_t gz;
    };

    /**
     * @brief Wait until frame synchronization is ready
     * 
     * @param timeoutMs Timeout in milliseconds
     * @return int Status code
     */
    int waitFrameSynchReady(int timeoutMs);


    /**
     * @brief Get gyroscope data
     * 
     * @param gx Reference to store X-axis gyro data
     * @param gy Reference to store Y-axis gyro data
     * @param gz Reference to store Z-axis gyro data
     * @return int Status code
     */
    int getData(int16_t& gx, int16_t& gy, int16_t& gz);
private:
    /**
     * @brief Initialize the I2C communication
     * 
     * @return int 
     */
    int initializeI2C();

    /**
     * @brief Initialize/configure the ICM-20948 after I2C is up
     *
     * @return int Status code
     */
    int initializeDevice();

    /**
     * @brief Read data from I2C device
     * 
     * @param reg Register address
     * @param data Pointer to data buffer
     * @param length Length of data to read
     * @return int Status code
     */
    int readI2CData(uint8_t reg, uint8_t* data, size_t length);

    /**
     * @brief Write data to I2C device
     * 
     * @param reg Register address
     * @param data Pointer to data buffer
     * @param length Length of data to write
     * @return int Status code
     */
    int writeI2CData(uint8_t reg, const uint8_t* data, size_t length);

    int setRegisterBankLocked(uint8_t bank);
    int readRegister(uint8_t bank, uint8_t reg, uint8_t* data, size_t length);
    int writeRegister(uint8_t bank, uint8_t reg, const uint8_t* data, size_t length);
    int clearDataReadyInterrupt();

    /**
     * @brief Read gyroscope data
     * 
     * @param gx Reference to store X-axis gyro data
     * @param gy Reference to store Y-axis gyro data
     * @param gz Reference to store Z-axis gyro data
     * @return int Status code
     */
    int readGyroData(GyroData& data);

    /**
     * @brief Poll gyro data in a separate thread
     * 
     */
    void pollGyroData(void);

    int initializeInterrupt(const char* irqChip, unsigned int irqLineOffset);
    int waitInterrupt(int timeoutMs);

    // Circular buffer
    Msg::CircularBuffer<GyroData> m_GyroBuffer{256};

    // I2C communication keep-alive flag
    std::atomic_bool m_ThreadCanRun{true};

    std::thread m_PollThread;

    // Close the I2C communication
    int fd_ = -1;

    // I2C device path
    const char* devicePath_;

    // I2C device file path
    const int gyroAddress_ = 0x68;  // I2C address for the gyro

    std::mutex m_I2cMutex;
    uint8_t m_CurrentBank = 0xFF;

    // Optional GPIO interrupt (data-ready) support via libgpiod
    struct gpiod_chip* m_IrqChip = nullptr;
    struct gpiod_line_request* m_IrqRequest = nullptr;
    struct gpiod_edge_event_buffer* m_IrqEventBuffer = nullptr;
};
}

#endif
