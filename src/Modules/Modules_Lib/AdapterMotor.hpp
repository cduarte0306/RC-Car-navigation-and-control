#pragma once
#ifndef ADAPTERMOTOR_HPP
#define ADAPTERMOTOR_HPP

#include "AdapterBase.hpp"

namespace Adapter {
    class MotorAdapter : public AdapterBase {
    public:

        // Motor module commands
        enum {
            MotorCmdSetSpeed,
            MotorCmdSteer,
            MotorCmdDisable,
            MotorCmdReadData,
            MotorCmdSpiWrite,
            MotorCmdSpiRead,
            MotorCmdWriteUpdate,
            MotorCmdInstallUpdate,
            MotorCmdReadVersion,
        };

        typedef struct {
            uint8_t command;
            val_type_t data_1;
            val_type_t data_2;
        } MotorCommand_t;

        MotorAdapter(std::string parentName_="");

        /**
         * @brief Set the Motor Speed
         * 
         * @param speed PWM setting
         * @return int 
         */
        int setMotorSpeed(int speed);

        /**
         * @brief Steer the motor
         * 
         * @param angle Steering angle
         * @return int Return status
         */
        int steer(int angle);

        /**
         * @brief Motor state control command
         * 
         * @param state Motor state
         * @return int 
         */
        int CommandMotorState(bool state);

    protected:
        // callable to request motor speed; empty when not set
        std::function<int(int )                     > motorSpeedCommand    = nullptr;
        std::function<int(int )                     > steerCommand         = nullptr;
        std::function<int(void)                     > disableMotorsCommand = nullptr;
        std::function<int(void)                     > enableMotorsCommand  = nullptr;
        std::function<int(char* data, size_t length)> getMotorStatus       = nullptr;
        std::function<int(void)                     > getDevice            = nullptr;

        virtual int bind_(AdapterBase* Adapter) override;

        void bindInterface(MotorAdapter* adapter);

        virtual int setMotorSpeed_(int direction);

        virtual int steer_(int counts);

    };
}

#endif // ADAPTERMOTOR_HPP
