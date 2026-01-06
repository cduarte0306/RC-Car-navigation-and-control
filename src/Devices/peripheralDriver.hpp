#ifndef  PERIPHERAL_DRIVER_HPP
#define  PERIPHERAL_DRIVER_HPP

#include "types.h"
#include <cstddef>
#include <stdint.h>

#include "DeviceBase.hpp"


namespace Device {
class PeripheralCtrl : public Device::DeviceBase {
public:
    typedef struct {
        val_type_t version_major;   // Major version
        val_type_t version_minor;   // Minor version
        val_type_t version_build;   // Build version
        val_type_t speed;          // Speed in Hz
        val_type_t frontDistance;  // Distance in mm
        val_type_t leftDistance;   // Distance in mm
        val_type_t rightDistance;  // Distance in mm
        val_type_t accelerationX; // Acceleration in X axis
        val_type_t accelerationY; // Acceleration in Y axis
        val_type_t accelerationZ; // Acceleration in Z axis
        val_type_t gyroX;         // Gyro data in X axis
        val_type_t gyroY;         // Gyro data in Y axis
        val_type_t gyroZ;         // Gyro data in Z axis
        val_type_t magneticX;     // Magnetic field in X axis
        val_type_t magneticY;     // Magnetic field in Y axis
        val_type_t magneticZ;     // Magnetic field in Z axis
        
        // Sensor health fields
        val_type_t sensorFStatus;  // Front sensor status
        val_type_t sensorLStatus;  // Left sensor status
        val_type_t sensorRStatus;  // Right sensor status

        val_type_t imuStatus;      // Gyro data in X axis
        val_type_t encoderStatus;  // Temperature in degree Celsius
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
    int doDetectDevice(void);
    int getVers(uint8_t& major, uint8_t& minor, uint8_t& build);
    int readData(psocDataStruct& data);
    int setMotorState(bool state);
    int setDriveMode(bool state);
    int setPIParams(float p, float i, float d);
    bool xferSPI(uint8_t* pbuf, size_t length);
    bool xfer(val_type_t* data, uint8_t reg);

private:
    enum
    {
        READ_TRANSACTION,          // Basic read transaction
        STAGE_RD_WRT_TRANSACTION,  // Stages a byte from the register map in the read buffer
        WRITE_REG_TRANSACTION      // Writes to selected register in register map
    };

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

        // IMU Registers
        REG_ACCEL_X,
        REG_ACCEL_Y,
        REG_ACCEL_Z,
        REG_GYRO_X,
        REG_GYRO_Y,
        REG_GYRO_Z,
        
        MAG_X,
        MAG_Y,
        MAG_Z,
        
        REG_TEMPERATURE,

        REG_SENSOR_F_STATUS,
        REG_SENSOR_L_STATUS,
        REG_SENSOR_R_STATUS,
        
        REG_ENCODER_STATUS,
        REG_IMU_STATUS,

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

    bool configSPI(void);  
private:
    bool isDeviceConnected_ = false;
    int spiFd = -1;
    uint32 speed = 0;
    uint8 bitsPerWord = 0;

    psocDataStruct psocData = {0};
};
}

#endif