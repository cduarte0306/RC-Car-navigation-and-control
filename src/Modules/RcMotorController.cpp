#include "RcMotorController.hpp"

#include <iostream>
#include <chrono>
#include <string>
#include <algorithm>
#include "utils/logger.hpp"
#include <nlohmann/json.hpp>
#include "Devices/RegisterMap.hpp"


namespace Modules {
MotorController::MotorController(int moduleID_, std::string name) : Base(moduleID, name), Adapter::MotorAdapter(name) {
    Logger* logger = Logger::getLoggerInst();

    try {
        this->peripheralDriver = std::make_unique<Device::PeripheralCtrl>();

        this->m_PwmFwd = std::make_unique<Device::Pwm>("/dev/pwm0", 1000);
        int ret = this->m_PwmFwd->writeEnable(true);
        if (ret < 0) {
            logger->log(Logger::LOG_LVL_ERROR, "Failed to enable Motor control PWM\r\n");
        }

        this->m_PwmSteer = std::make_unique<Device::Pwm>("/dev/pwm2", 50);
        ret = this->m_PwmSteer->writeEnable(true);
        if (ret < 0) {
            logger->log(Logger::LOG_LVL_ERROR, "Failed to enable Servo control PWM\r\n");
        }
    } catch (const std::exception& e) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to initialize PeripheralCtrl and motor control: %s\r\n", e.what());
    }

    // Open GPIO for enabling motor direction
    m_GpioEnable = Device::Gpio::create(31); // Physical header pin 33, GPIO1_31
}

MotorController::~MotorController() {
    this->m_PwmFwd.reset();
    this->m_PwmSteer.reset();
    this->peripheralDriver.reset();
}


/**
 * @brief Initialize the motor controller module
 * 
 * @return int Error code
 */
int MotorController::init(void) {
    // Open a network adapter for telemetry
    Logger* logger = Logger::getLoggerInst();

    // Register as telemetry source
    int ret = this->TlmAdapter->registerTelemetrySource(this->getName());
    if (ret < 0) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to register motor controller as telemetry source\r\n");
        return -1;
    }

    logger->log(Logger::LOG_LVL_INFO, "Opened motor telemetry network adapter at port 65001\r\n");
    return 0;
}


/**
 * @brief Serial-like interface for motor controller module
 * 
 * @param pbuf 
 * @param len 
 * @return int 
 */
int MotorController::moduleCommand_(char* pbuf, size_t len) {
    Logger* logger = Logger::getLoggerInst();
    logger->log(Logger::LOG_LVL_INFO, "Module command received\r\n");
    MotorCommand_t* cmd = reinterpret_cast<MotorCommand_t*>(pbuf);
    char* payload = reinterpret_cast<char*>(pbuf + sizeof(MotorCommand_t));
    bool ret = false;
    switch (cmd->command)
    {
    case MotorCmdSetSpeed:
        break;

    case MotorCmdSteer:
        /* code */
        break;

    case MotorCmdDisable:
        /* code */
        break;

    case MotorCmdReadData:
        {
        std::lock_guard<std::mutex> lock(mtrControllerMutex);
        std::memcpy(payload, &psocData, sizeof(psocData));
        }
        break;

    case MotorCmdSpiWrite:
        logger->log(Logger::LOG_LVL_INFO, "SPI WRITE: Reg: %u Data: %u\r\n", cmd->data_1.u8, cmd->data_2.u32);
        {
            std::lock_guard<std::mutex> lock(mtrControllerMutex);
            ret = this->peripheralDriver->xferSPI(reinterpret_cast<uint8_t*>(&cmd->data_1), sizeof(val_type_t));
        }
        if (!ret) {
            logger->log(Logger::LOG_LVL_ERROR, "SPI WRITE failed\r\n");
            return -1;
        }
        break;

    case MotorCmdSpiRead:
        logger->log(Logger::LOG_LVL_INFO, "SPI READ: Reg: %u\r\n", cmd->data_1.u8);
        {
            std::lock_guard<std::mutex> lock(mtrControllerMutex);
            ret = this->peripheralDriver->xferSPI(reinterpret_cast<uint8_t*>(&cmd->data_1), sizeof(val_type_t));
        }
        if (!ret) {
            logger->log(Logger::LOG_LVL_ERROR, "SPI READ failed\r\n");
            return -1;
        }
        break;
    
    default:
        break;
    }

    // Implementation for processing module commands
    return 0;
}


/**
 * @brief Set the motor speed
 * 
 * @param speed Speed in PWM duty cycle
 * @return int 
 */
int MotorController::setMotorSpeed_(int speed) {
    if (!m_isControllerConnected) {
        return -1;
    }

    if (speed < 5 && speed > -5) {
        speed = 0; // Deadzone
    }

    if (speed < 0) {
        m_GpioEnable->gpioWrite(0); // Set direction GPIO high for reverse
        speed = -speed;
    } else {
        m_GpioEnable->gpioWrite(1); // Set direction GPIO low for forward
    }

    // Convert to Duty cycle percentage (0-100). Analog stick input range is -127 to 127
    int dutyCycle = (static_cast<int>((static_cast<float>(speed) / 127.0f) * 100.0f));
    speed = std::min(100, std::max(0, dutyCycle));

    Logger* logger = Logger::getLoggerInst();
    // int ret =0;
    int ret = this->m_PwmFwd->writeDutyCycle(speed);
    if (ret < 0) {
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
    if (ret < 0) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_ERROR,
            "Failed to set PWM duty cycle\r\n");
        return -1;
    }

    return 0;
}


/**
 * @brief Polls telemetry data
 * 
 */
void MotorController::pollTlmData(void) {
    {
        std::lock_guard<std::mutex> lock(mtrControllerMutex);
        this->peripheralDriver->readData(psocData);
    }
    RegisterMap* regMap = RegisterMap::getInstance();
    auto retVal = regMap->get<std::string>(RegisterMap::RegisterKeys::HostIP);

    nlohmann::json telemetryJson;

    if (m_isControllerConnected) {
        telemetryJson["status"]        = true;
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

    if (retVal.has_value()) {
        this->TlmAdapter->publishTelemetry(this->getName(),
            reinterpret_cast<const uint8_t*>(telemetryJson.dump().c_str()),
            telemetryJson.dump().length());
    }
}


void MotorController::mainProc() {
    Logger* logger = Logger::getLoggerInst();
    m_isControllerConnected = false;

    // Main processing loop for the motor controller
    while (m_Running.load()) {
        if (!m_isControllerConnected) {
            this->peripheralDriver->doDetectDevice();
            int ret = this->peripheralDriver->doDetectDevice();
            if (ret == 0) {
                uint8_t major, minor, build;
                this->peripheralDriver->getVers(major, minor, build);
                logger->log(Logger::LOG_LVL_INFO, "PSoC Version detected: %u.%u.%u\r\n", major, minor, build);
                m_isControllerConnected = true;
            }
        } else {
            pollTlmData();
        }

        // Process motor commands
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
}
