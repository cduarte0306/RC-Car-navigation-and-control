#include "RcMotorController.hpp"

#include <iostream>
#include <chrono>
#include <string>
#include <algorithm>
#include <stdlib.h>
#include "utils/logger.hpp"
#include <nlohmann/json.hpp>
#include "app/motor/MotorLogController.hpp"
#include "lib/RegisterMap.hpp"

#define SLEEP_1SEC usleep(1000000)

namespace Modules {
MotorController::MotorController(ModuleDefs::DeviceType moduleID_, std::string name) : Base(moduleID_, name), Adapter::MotorAdapter(name)
{
    Logger* logger = Logger::getLoggerInst();

    setInputAdapter(static_cast<Adapter::AdapterBase*>(static_cast<Adapter::MotorAdapter*>(this)));

    try
    {
        this->peripheralDriver = std::make_unique<Device::PeripheralCtrl>();

        this->m_PwmFwd = std::make_unique<Device::Pwm>("/dev/pwm0", 1000);
        int ret = this->m_PwmFwd->writeEnable(true);
        if (ret < 0)
        {
            logger->log(Logger::LOG_LVL_ERROR, "Failed to enable Motor control PWM\r\n");
        }

        this->m_PwmSteer = std::make_unique<Device::Pwm>("/dev/pwm2", 50);
        ret = this->m_PwmSteer->writeEnable(true);
        if (ret < 0)
        {
            logger->log(Logger::LOG_LVL_ERROR, "Failed to enable Servo control PWM\r\n");
        }
    }
    catch (const std::exception& e)
    {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to initialize PeripheralCtrl and motor control: %s\r\n", e.what());
    }

    // Open GPIO for enabling motor direction
    m_GpioEnable = Device::Gpio::create(31); // Physical header pin 33, GPIO1_31
}

MotorController::~MotorController()
{
    this->m_PwmFwd.reset();
    this->m_PwmSteer.reset();
    this->peripheralDriver.reset();
}


/**
 * @brief Initialize the motor controller module
 * 
 * @return int Error code
 */
int MotorController::init(void)
{
    // Open a network adapter for telemetry
    Logger* logger = Logger::getLoggerInst();

    // Register as telemetry source
    int ret = this->TlmAdapter->registerTelemetrySource(this->getName());
    if (ret < 0)
    {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to register motor controller as telemetry source\r\n");
        return -1;
    }

    // Register commands,
    Base::moduleRegisterCommand(MotorCmdSetSpeed, &MotorController::cmdHandlerSetSpeed);
    Base::moduleRegisterCommand(MotorCmdSteer, &MotorController::cmdHandlerSteer);
    Base::moduleRegisterCommand(MotorCmdDisable, &MotorController::cmdHandlerDisable);
    Base::DefinePayloadLoc(sizeof(MotorCommand_t));  // Define the payload location and size for incoming commands

    logger->log(Logger::LOG_LVL_INFO, "Opened motor telemetry network adapter at port 65001\r\n");
    return 0;
}


int MotorController::stop(void)
{
    Logger* logger = Logger::getLoggerInst();
    logger->log(Logger::LOG_LVL_INFO, "Stopping motor operations...\r\n");
    peripheralDriver->setDriveMode(false);   // Set to manual
    peripheralDriver->setMotorState(false);  // Disable motor
    return 0;
}


/**
 * @brief Set the motor speed
 * 
 * @param speed Speed in PWM duty cycle
 * @return int 
 */
int MotorController::setMotorSpeed_(int speed)
{
    if (!m_isControllerConnected)
    {
        return -1;
    }

    if (speed < 5 && speed > -5)
    {
        speed = 0; // Deadzone
    }

    if (speed < 0)
    {
        m_GpioEnable->gpioWrite(0); // Set direction GPIO high for reverse
        speed = -speed;
    }
    else
    {
        m_GpioEnable->gpioWrite(1); // Set direction GPIO low for forward
    }

    // Convert to Duty cycle percentage (0-100). Analog stick input range is -127 to 127
    int dutyCycle = (static_cast<int>((static_cast<float>(speed) / 127.0f) * 100.0f));
    speed = std::min(100, std::max(0, dutyCycle));

    Logger* logger = Logger::getLoggerInst();
    // int ret =0;
    int ret = this->m_PwmFwd->writeDutyCycle(speed);
    if (ret < 0)
    {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to set PWM duty cycle: %d\r\n", speed);
    }
    return ret;
}


/**
 * @brief Steers the motor 
 * 
 * @param angle Accetps steer in counts
 * @return int 
 */
int MotorController::steer_(int counts)
{
    if (!m_isControllerConnected)
        return -1;

    // Clamp
    counts = std::max(-127, std::min(127, counts));

    // Map -127..127 -> 5%..10%
    float duty_percent = 7.5f + (counts / 127.0f) * 2.5f;

    // Convert to integer for ioctl
    uint32_t duty_int = static_cast<uint32_t>(duty_percent);

    int ret = this->m_PwmSteer->writeDutyCycle(duty_int);
    if (ret < 0)
    {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR,
            "Failed to set PWM duty cycle\r\n");
        return -1;
    }

    return 0;
}

void MotorController::cmdHandlerSetSpeed(val_type_t val, const std::vector<char>& payload)
{
    (void)payload;
    if (setMotorSpeed_(val.i32) < 0)
    {
        Base::DoAck(false, {});
    }
    return;
}

void MotorController::cmdHandlerSteer(val_type_t val, const std::vector<char>& payload)
{
    (void)payload;
    if (steer_(val.i32) < 0)
    {
        Base::DoAck(false, {});
    }
    return;
}

void MotorController::cmdHandlerDisable(val_type_t val, const std::vector<char>& payload)
{
    (void)payload;
    Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "Disable command received: %d\r\n", val.u8);
    if (val.u8)
    {
        if (stop() < 0)
        {
            Base::DoAck(false, {});
        }
    }
    return;
}


/**
 * @brief Polls telemetry data
 * 
 */
void MotorController::pollTlmData(void)
{
    {
        std::lock_guard<std::mutex> lock(mtrControllerMutex);
        if (this->peripheralDriver->readData(psocData) < 0)
        {
            throw Device::PeripheralDisconnectHandler();
            return;
        }
    }
    RegisterMap* regMap = RegisterMap::getInstance();

    nlohmann::json telemetryJson;

    if (m_isControllerConnected)
    {
        telemetryJson["status"       ] = true;
        telemetryJson["version_major"] = psocData.version_major.u8;
        telemetryJson["version_minor"] = psocData.version_minor.u8;
        telemetryJson["version_build"] = psocData.version_build.u8;
        telemetryJson["speed"        ] = psocData.speed.u32;
        telemetryJson["frontDistance"] = psocData.frontDistance.u32;
        telemetryJson["leftDistance" ] = psocData.leftDistance.u32;
        telemetryJson["rightDistance"] = psocData.rightDistance.u32;
        telemetryJson["accelerationX"] = psocData.accelerationX.f32;
        telemetryJson["accelerationY"] = psocData.accelerationY.f32;
        telemetryJson["accelerationZ"] = psocData.accelerationZ.f32;
        telemetryJson["gyroX"        ] = psocData.gyroX.f32;
        telemetryJson["gyroY"        ] = psocData.gyroY.f32;
        telemetryJson["gyroZ"        ] = psocData.gyroZ.f32;
        telemetryJson["magneticX"    ] = psocData.magneticX.f32;
        telemetryJson["magneticY"    ] = psocData.magneticY.f32;
        telemetryJson["magneticZ"    ] = psocData.magneticZ.f32;
        telemetryJson["sensorFStatus"] = psocData.sensorFStatus.u8;
        telemetryJson["sensorLStatus"] = psocData.sensorLStatus.u8;
        telemetryJson["sensorRStatus"] = psocData.sensorRStatus.u8;
        telemetryJson["imuStatus"    ] = psocData.imuStatus.u8;
        telemetryJson["encoderStatus"] = psocData.encoderStatus.u8;
    }

    this->TlmAdapter->publishTelemetry(this->getName(),
        reinterpret_cast<const uint8_t*>(telemetryJson.dump().c_str()),
        telemetryJson.dump().length());
}


void MotorController::mainProc()
{
    typedef enum
    {
        eRunning,
        eCheckMotorState,
        eMotorInitialize,
        eMotorProblem,
        eVerifyConnection,
        eMotorIdle
    } tMotorCtrlState;

    tMotorCtrlState mtrState, prevState = eVerifyConnection;

    int ret = 0;
    Logger* logger = Logger::getLoggerInst();
    m_isControllerConnected = false;
    Motor::MotorLogController logController; // Start the motor log controller to capture low-level logs from the motor controller

    // Main processing loop for the motor controller
    while (m_Running.load())
    {
        prevState = mtrState;
        switch(mtrState)
        {
            case eVerifyConnection:
                ret = this->peripheralDriver->doDetectDevice();
                if (ret == 0)
                {
                    uint8_t major, minor, build;
                    this->peripheralDriver->getVers(major, minor, build);
                    logger->log(Logger::LOG_LVL_INFO, "PSoC Version detected: %u.%u.%u\r\n", major, minor, build);
                    m_isControllerConnected = true;
                    peripheralDriver->setMotorState(true);
                    mtrState = eCheckMotorState;
                }
                else
                {
                    logger->log(Logger::LOG_LVL_INFO, "Failed to detect PSoC device\r\n");
                    SLEEP_1SEC;  // Wait 1 second before trying again
                }
                break;
            case eCheckMotorState:
                logger->log(Logger::LOG_LVL_INFO, "Checking motor state\r\n");
                {
                    val_type_t val;
                    ret = peripheralDriver->readReg(Device::PeripheralCtrl::REG_MOTOR_ONOFF_STATE, val);
                    if (ret < 0)
                    {
                        logger->log(Logger::LOG_LVL_INFO, "Failed to read register: %02X\r\n", 
                                    Device::PeripheralCtrl::REG_MOTOR_ONOFF_STATE);
                        mtrState = eMotorIdle;
                        break;
                    }
                    if (val.u8)
                    {
                        // Motor is now running
                        logger->log(Logger::LOG_LVL_INFO, "Motor now running\r\n");
                        mtrState = eRunning;
                    }
                    else
                    {
                        logger->log(Logger::LOG_LVL_INFO, "Motor not yet in running state\r\n");
                        mtrState = eMotorInitialize;
                        SLEEP_1SEC;
                        break;
                    }
                }
                break;
            case eMotorInitialize:
                logger->log(Logger::LOG_LVL_INFO, "Initializing motor\r\n");
                peripheralDriver->setMotorState(true);
                SLEEP_1SEC;  // Allow 1 second for device to initialize
                mtrState = eCheckMotorState;
                break;
            case eRunning:
                try
                {
                    pollTlmData();   
                }
                catch (Device::PeripheralDisconnectHandler& e)
                {
                    logger->log(Logger::LOG_LVL_ERROR, "Peripheral disconnected: %s\r\n", e.what());
                    mtrState = eMotorProblem;
                }
                break;
            case eMotorProblem:
                usleep(1000000);
                mtrState = eVerifyConnection;
                break;
            case eMotorIdle:
                usleep(1000000);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
}
