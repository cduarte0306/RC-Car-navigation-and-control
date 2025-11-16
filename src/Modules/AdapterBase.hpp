#pragma once

#include <map>
#include <string>
#include <unordered_map>
#include <list>

#include <functional>
#include <iostream>


namespace Adapter {
    class AdapterBase {
    public:
        AdapterBase(std::string parentName_="") {}
        virtual ~AdapterBase() {}

        virtual int bind(AdapterBase* Adapter)= 0;

        /**
         * Default handler for moduleCommand. Modules or adapters that need to
         * provide custom handling should override this. Providing a non-pure
         * implementation here allows concrete adapter types to be instantiated
         * without requiring every adapter to implement moduleCommand.
         */
        virtual int moduleCommand(char* pbuf, size_t len) {
            if (!moduleWriteCmd) {
                std::cout << "Module command error\r\n";
                return -1;
            }

            std::cout << "Writing module command\r\n";
            moduleWriteCmd(pbuf, len);
            return 0;
        }

        /**
         * @brief Add a module name to the bound modules list
         * 
         * @param moduleName Name of the module to add
         */
        void addAdapter(std::string moduleName) {
            boundModules.push_back(moduleName);
        }

        /**
         * @brief Get the parent name
         * 
         * @return std::string parent module name 
         */
        std::string getParentName() const {
            return parentName;
        }
    protected:
        std::string parentName;
        std::list<std::string> boundModules;
        std::unordered_map<std::string, AdapterBase*> adapterMap;
        std::function< int(char* pbuf, size_t len) > moduleWriteCmd = nullptr;

        virtual int moduleCommand_(char* pbuf, size_t len) {
            (void)pbuf; (void)len;
            return -1;
        }
    };

    class MotorAdapter : public AdapterBase {
    public:
        MotorAdapter(std::string parentName_="") : AdapterBase(parentName_) {}

        virtual int bind(AdapterBase* Adapter) override {
            // Implementation for binding motor adapter
            bindInterface(static_cast<MotorAdapter*>(Adapter));
            return 0;
        }

        /**
         * @brief Set the Motor Speed
         * 
         * @param speed PWM setting
         * @return int 
         */
        int setMotorSpeed(int speed) {
            // Implementation for setting motor speed
            if (!motorSpeedCommand) {
                return -1; // No command set
            }

            return motorSpeedCommand(speed);
        }

        /**
         * @brief Steer the motor
         * 
         * @param angle Steering angle
         * @return int Return status
         */
        int steer(int angle) {
            // Implementation for steering
            if (!steerCommand) {
                return -1; // No command set
            }

            steerCommand(angle);
            return 0;
        }

    protected:
        // callable to request motor speed; empty when not set
        std::function<int(int )                     > motorSpeedCommand    = nullptr;
        std::function<int(int )                     > steerCommand         = nullptr;
        std::function<int(void)                     > disableMotorsCommand = nullptr;
        std::function<int(char* data, size_t length)> getMotorStatus       = nullptr;
        std::function<int(void)                     > getDevice            = nullptr;

        void bindInterface(MotorAdapter* adapter) {
            if (!adapter) return;
            // When binding, expose this adapter's implementation to the target
            // adapter by assigning callables on the target that call into
            // this instance's protected implementation methods.
            // When binding to another adapter instance, expose the other
            // adapter's implementation as the callable targets for this
            // adapter. That way, calling this->setMotorSpeed(...) will
            // forward into the bound adapter's implementation.
            this->motorSpeedCommand = [adapter](int speed) {
                return adapter->setMotorSpeed_(speed);
            };

            this->steerCommand = [adapter](int angle) {
                return adapter->steer_(angle);
            };

            this->moduleWriteCmd = [adapter](char* pbuf, size_t len) {
                return adapter->moduleCommand_(pbuf, len);
            };

            // record the mapping for lookup if needed
            adapterMap[adapter->getParentName()] = adapter;
        }

        virtual int setMotorSpeed_(int direction) {
            // Implementation for setting motor direction
            return 0;
        }

        virtual int steer_(int counts) {
            // Implementation for steering
            return 0;
        }
    };

    class CameraAdapter : public AdapterBase {
    public:
        CameraAdapter() {}

        virtual int bind(AdapterBase* Adapter) override {
            // Implementation for binding motor adapter
            return 0;
        }
    };

    class CommsAdapter : public AdapterBase {
    public:
        CommsAdapter() {}

        virtual int bind(AdapterBase* Adapter) override {
            // Implementation for binding motor adapter
            return 0;
        }

        virtual int transmitData(const uint8_t* data, size_t length) {
            if (!data || length == 0) {
                return -1;
            }
            
            return 0;
        }

    protected:
        // callable to request motor speed; empty when not set
        std::function<int(const uint8_t*, size_t)> transmitDataCommand    = nullptr;

        void bindInterface(CommsAdapter* adapter) {
            if (!adapter) return;
            // bind this instance's implementation as a callable on the target adapter
            adapter->transmitDataCommand = [this](const uint8_t* pData, size_t length) -> int {
                return this->transmitData_(pData, length);
            };
        }

        virtual int transmitData_(const uint8_t* data, size_t length) {
            if (!data || length == 0) {
                return -1;
            }
            
            return 0;
        }
    };
}

#pragma endregion