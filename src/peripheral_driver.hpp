#ifndef  PERIPHERAL_DRIVER_HPP
#define  PERIPHERAL_DRIVER_HPP

#include "types.h"
#include <cstddef>
#include <stdint.h>
#include <thread>


class PeripheralCtrl {
public:
    typedef struct {
        val_type_t version_major;   // Major version
        val_type_t version_minor;   // Minor version
        val_type_t version_build;   // Build version
        val_type_t speed;          // Speed in Hz
        val_type_t frontDistance;  // Distance in mm
        val_type_t leftDistance;   // Distance in mm
        val_type_t rightDistance;  // Distance in mm
    } psocDataStruct;

    typedef enum
    {
        MANUAL,
        AUTOMATIC,
    } driveModes;
public:
    PeripheralCtrl();
    ~PeripheralCtrl();

    bool isDeviceConnected(void) const { return this->isDeviceConnected_; }
    bool doConfigureDevice(void);
    bool doDetectDevice(uint32_t& version);
    int readData(psocDataStruct& data);
    int setMotorState(bool state);
    int setDriveMode(bool state);
    int setPIParams(float p, float i, float d);

private:
    typedef enum
    {
        REG_NOOP,
        REG_VER_MAJOR,
        REG_VER_MINOR,
        REG_VER_BUILD,
        REG_SPEED,
        REG_FRONT_DISTANCE,
        REG_LEFT_DISTANCE,
        REG_RIGHT_DISTANCE,
        REG_RO_END
    } registerEnumsReadOnly;

    typedef enum
    {
        REG_SET_MOTOR_STATUS = REG_RO_END,
        REG_SPEED_SETPOINT,
        REG_PID_P,
        REG_PID_I,
        REG_PID_D,
        REG_WR_END,
    } registerEnumReadWrites;

    typedef struct __attribute__((__packed__))
    {
        uint8_t transactionType;
        uint8_t reg;
        val_type_t data;
        uint8_t ack;
    } spiTransactionStruct;

    bool xfer(val_type_t* data, uint8_t reg);
    bool configSPI(void);  
    bool configSerial(void);
    bool xferSPI(uint8_t* pbuf, size_t length);

#if !defined(__aarch64__)
    

    bool wrtReg(val_type_t data, uint8_t reg);
    bool rdReg(val_type_t& data, uint8_t reg);

    bool wrtSerial(val_type_t data, uint8_t reg);
    bool rdSerial(val_type_t& data, uint8_t reg);
#endif

private:
    bool isDeviceConnected_ = false;
    int spiFd = -1;
    uint32 speed = 0;
    uint8 bitsPerWord = 0;

    psocDataStruct psocData = {0};
    
#if !defined(__aarch64__)
    std::thread serCommsThread;
    int serialFd = -1;  // File descriptor for serial communication
    char serialBuffer[1024] = {0};  // Buffer for serial communication
#endif
};

#endif