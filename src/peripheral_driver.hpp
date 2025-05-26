#ifndef  PERIPHERAL_DRIVER_HPP
#define  PERIPHERAL_DRIVER_HPP

#include "types.h"
#include <cstddef>
#include <stdint.h>


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

public:
    PeripheralCtrl();
    ~PeripheralCtrl();

    bool isDeviceConnected(void) const { return this->isDeviceConnected_; }
    bool doConfigureDevice(void);
    bool doDetectDevice(void);
    int readData(psocDataStruct& data);

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
    } registerEnums;

    typedef struct __attribute__((__packed__))
    {
        uint8_t transactionType;
        uint8_t reg;
        val_type_t data;
        uint8_t ack;
    } spiTransactionStruct;

    bool xfer(val_type_t* data, uint8_t reg);
    bool configSPI(void);  
    bool xferSPI(uint8_t* pbuf, size_t length);

private:
    bool isDeviceConnected_ = false;
    int spiFd = -1;
    uint32 speed = 0;
    uint8 bitsPerWord = 0;

    psocDataStruct psocData = {0};
};

#endif