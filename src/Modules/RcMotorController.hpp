#include <thread>
#include <mutex>
#include <atomic>
#include "AdapterBase.hpp"
#include "RcBase.hpp"
#include "RcMessageLib.hpp"
#include "Devices/peripheralDriver.hpp"
#include "Devices/DeviceBase.hpp"

namespace Modules {
class MotorController : public Modules::Base, public Adapter::MotorAdapter {
public:
    MotorController(int moduleID_, std::string name);
    ~MotorController();

    virtual int stop(void) override {
        // Implementation to stop the motor controller
        return 0;
    }


    Adapter::AdapterBase* getInputAdapter() override {
        return static_cast<Adapter::AdapterBase*>(static_cast<Adapter::MotorAdapter*>(this));
    }


    Device::DeviceBase* getDevice() {
        return this->peripheralDriver.get();
    }

    enum {
        MOTOR_CMD_SET_SPEED,
        MOTOR_CMD_STEER,
        MOTOR_CMD_DISABLE,
        MOTOR_CMD_GET_STATUS,
        MOTOR_CMD_READ_DATA,
        MOTOR_CMD_SPI_WRITE,
        MOTOR_CMD_SPI_READ
    };

    typedef struct {
        uint8_t command;
        val_type_t data_1;
        val_type_t data_2;
    } MotorCommand_t;
protected:
    enum {
        CMD_NOOP,
        CMD_FWD_DIR,
        CMD_STEER,
    };

    virtual void mainProc() override;

    virtual int moduleCommand_(char* pbuf, size_t len) override;

    /**
     * @brief Transmit telemetry data to the host
     * 
     */
    void pollTlmData(void);

    /**
     * @brief Set motor speed implementation
     * 
     * @param speed Motor speed
     * @return int Return status
     */
    virtual int setMotorSpeed_(int speed) override {
        if (!m_isControllerConnected) {
            return -1;
        }
        // return this->peripheralDriver->setMotorState(speed != 0);
        return 0;
    }

    /**
     * @brief Peripheral driver instance
     * 
     */
    std::unique_ptr<Device::PeripheralCtrl> peripheralDriver = nullptr;

    /**
     * @brief Is the motor controller connected
     * 
     */
    bool m_isControllerConnected = false;

    /**
     * @brief PSoC data buffer
     * 
     */
    Device::PeripheralCtrl::psocDataStruct psocData;

    std::mutex mtrControllerMutex;
};
} // namespace Modules