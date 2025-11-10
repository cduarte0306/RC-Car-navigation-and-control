#pragma once

#include <map>
#include <string>
#include <unordered_map>
#include <list>

#include <functional>

namespace Adapter {
    class AdapterBase {
    public:
        AdapterBase(std::string parentName_="") {}
        virtual ~AdapterBase() {}

        virtual int bind(AdapterBase* Adapter)= 0;

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
    private:
        std::list<std::string> boundModules;
    };

    class MotorAdapter : public AdapterBase {
    public:
        MotorAdapter(std::string parentName_="") : AdapterBase(parentName_) {}

        virtual int bind(AdapterBase* Adapter) override {
            // Implementation for binding motor adapter
            bindInterface(static_cast<MotorAdapter*>(Adapter));
            return 0;
        }

        int setMotorSpeed(int speed) {
            // Implementation for setting motor speed
            if (!motorSpeedCommand) {
                return -1; // No command set
            }

            motorSpeedCommand(speed);
            return 0;
        }

        int steer(int angle) {
            // Implementation for steering
            if (!steerCommand) {
                return -1; // No command set
            }

            steerCommand(angle);
            return 0;
        }

    protected:
        std::unordered_map<std::string, MotorAdapter*> adapterMap;
        // callable to request motor speed; empty when not set
        std::function<int(int )> motorSpeedCommand    = nullptr;
        std::function<int(int )> steerCommand         = nullptr;
        std::function<int(void)> disableMotorsCommand = nullptr;

        void bindInterface(MotorAdapter* adapter) {
            if (!adapter) return;
            // bind this instance's implementation as a callable on the target adapter
            adapter->motorSpeedCommand = [this](int speed) {
                return this->setMotorSpeed_(speed);
            };

            adapter->steerCommand = [this](int angle) {
                return this->steer_(angle);
            };

            adapterMap[adapter->getParentName()] = adapter;
        }

        int setMotorSpeed_(int direction) {
            // Implementation for setting motor direction
            return 0;
        }

        int steer_(int angle) {
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
    }; 
}

#pragma endregion