#ifndef  PERIPHERAL_DRIVER_HPP
#define  PERIPHERAL_DRIVER_HPP

#include "types.h"
#include <cstddef>
#include <stdint.h>


class PeripheralCtrl {
public:
    typedef enum
    {
        REG_NOOP,
        REG_SPEED,
        REG_FRONT_DISTANCE,
        REG_LEFT_DISTANCE,
        REG_RIGHT_DISTANCE,
        REG_END
    } registerEnums;

public:
    PeripheralCtrl();
    ~PeripheralCtrl();

    bool doConfigureDevice(void);
    bool doDetectDevice(void);
    bool xfer(val_type_t* data);

private:
    typedef struct
    {
        uint8_t transactionType;
        uint8_t reg;
        val_type_t data;
        uint8_t ack;
    } spiTransactionStruct;

    bool configSPI(void);  
    bool xferSPI(uint8_t* pbuf, size_t length);

private:
    int spiFd = -1;
    uint32 speed = 0;
    uint8 bitsPerWord = 0;
};

#endif