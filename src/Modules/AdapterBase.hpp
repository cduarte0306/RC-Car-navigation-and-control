#pragma once

#include <map>
#include <string>
#include <unordered_map>
#include <list>
#include <utility>

#include <functional>
#include <iostream>


namespace Adapter {
    enum {
        CommsAdapterID = 1,
        CommandAdapterID,
        MotorAdapterID, 
        CameraAdapterID
    };

    class AdapterBase {
    public:
        AdapterBase(int id, std::string parentName_="") : parentName(parentName_), adapterId(id) {}
        virtual ~AdapterBase() {}

        int bind(AdapterBase* Adapter) {
            // record the mapping for lookup if needed
            adapterMap[Adapter->getParentName()] = Adapter;
            return bind_(Adapter);
        }

        virtual int bind_(AdapterBase* Adapter)= 0;

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
        std::function< int(char* pbuf, size_t len) > moduleWriteAsyncCmd = nullptr;

        const int adapterId;

        virtual int moduleCommand_(char* pbuf, size_t len) {
            (void)pbuf; (void)len;
            return -1;
        }

        virtual int moduleCommandAsync_(char* pbuf, size_t len) {
            (void)pbuf; (void)len;
            return -1;
        }
    };

    class MotorAdapter : public AdapterBase {
    public:
        MotorAdapter(std::string parentName_="") : AdapterBase(MotorAdapterID, parentName_) {}

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

        virtual int bind_(AdapterBase* Adapter) override {
            // Implementation for binding motor adapter
            bindInterface(static_cast<MotorAdapter*>(Adapter));
            return 0;
        }


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
        CameraAdapter(std::string parentName_="") : AdapterBase(CameraAdapterID, parentName){}

        int setCameraState(bool state) {
            return 0;
        }

        virtual int configurePipeline(const std::string& host) {
            if (!configurePipelineCommand) {
                return -1;
            }

            return this->configurePipelineCommand(host);
        }
    private:
        std::function<int(bool)>                    setCameraStateCommand    = nullptr;
        std::function<int(const std::string& host)> configurePipelineCommand = nullptr;

        virtual int bind_(AdapterBase* Adapter) override {
            // Implementation for binding motor adapter
            bindInterface(static_cast<CameraAdapter*>(Adapter));
            return 0;
        }

        void bindInterface(CameraAdapter* adapter) {
            if (!adapter) return;
            // bind this instance's implementation as a callable on the target adapter
            this->setCameraStateCommand = [adapter](bool state) -> int {
                return adapter->setCameraState_(state);
            };

            this->configurePipelineCommand = [adapter](const std::string& host) -> int {
                return adapter->configurePipeline_(host);
            };
        }

        virtual int setCameraState_(int direction) {
            // Implementation for setting motor direction
            return 0;
        }

        virtual int configurePipeline_(const std::string& host) {
            // Implementation for setting motor direction
            return 0;
        }
    };


    class CommsAdapter : public AdapterBase {
    public:
        CommsAdapter(std::string parentName_="") : AdapterBase(CommsAdapterID, parentName_) {}

        virtual int transmitData(const uint8_t* data, size_t length) {
            if (!data || length == 0) {
                return -1;
            }

            return transmitDataCommand(this->parentName, data, length);
        }

        virtual int openAdapter(int port, std::string& adapter) {
            return openAdapterCommand(parentName, port, adapter);
        }

    protected:
        // callable to request data transmit; now includes caller identity
        std::function<int(const std::string& caller, const uint8_t*, size_t)> transmitDataCommand = nullptr;
        std::function<int(std::string&, int, std::string&)> openAdapterCommand  = nullptr;
        std::function<int (const char* pbuf, size_t len)> recvDataCallback      = nullptr;

        std::list<std::pair<std::string, std::string>> m_RegisteredCallers;  // List of modules that have opened an adapter here
        // Fast lookup from caller module name -> adapter pointer (populated on open)
        std::unordered_map<std::string, AdapterBase*> m_CallerAdapterMap;

        virtual int bind_(AdapterBase* Adapter) final {
            bindInterface(static_cast<CommsAdapter*>(Adapter));
            return 0;
        }

        void bindInterface(CommsAdapter* adapter) {            
            if (!adapter) return;

            // bind this instance's implementation as a callable on the target adapter
            // include the caller identity so the controller can route efficiently
            adapter->transmitDataCommand = [this](const std::string& caller, const uint8_t* pData, size_t length) -> int {
                return this->transmitData_(caller, pData, length);
            };

            adapter->openAdapterCommand = [this](std::string& parent, int port, std::string& adapter) -> int {
                return this->openAdapter_(parent, port, adapter);
            };
        }

        virtual int transmitData_(const std::string& caller, const uint8_t* data, size_t length) {
            (void)caller;
            if (!data || length == 0) {
                return -1;
            }

            // Default implementation doesn't know how to route - subclasses
            // (e.g. a network comms controller) should override this method
            // and use `caller` to pick the fastest transmit path.
            return 0;
        }


        virtual int openAdapter_(std::string& parent, int port, std::string& adapter) final {
            bool found = false;
            for (const auto& [callerName, adp] : m_RegisteredCallers) {
                if (parent == callerName) {
                    found = true;
                    break;
                }
            }

            if (found) {
                std::string msg("Adapter " + adapter + " from parent " + parent + " already exists\n");
                throw(msg);
            }

            std::pair<std::string, std::string> adapterDesc(parent, adapter);
            m_RegisteredCallers.push_back(adapterDesc);

            // If we have previously bound this caller adapter into adapterMap
            // (via AdapterBase::bind), cache a direct pointer for fast lookup
            auto it = adapterMap.find(parent);
            if (it != adapterMap.end()) {
                m_CallerAdapterMap[parent] = it->second;
            }

            return 0;
        }

        virtual int configureAdapter(int port, std::string& adapter) {
            return 0;
        }
    };


    class CommandAdapter : public AdapterBase {
    public:
        CommandAdapter(std::string parentName_="") : AdapterBase(CommandAdapterID, parentName_) {}

        virtual int transmitData(const uint8_t* data, size_t length) {
            if (!data || length == 0) {
                return -1;
            }

            return transmitDataCommand(data, length);
        }

        virtual int openAdapter(int port, std::string adapter) {
            return 0;
        }

    protected:
        // callable to request motor speed; empty when not set
        std::function<int(const uint8_t*, size_t)> transmitDataCommand = nullptr;
        std::function<int(const uint8_t*, size_t)> openAdapterCommand  = nullptr;

        virtual int bind_(AdapterBase* Adapter) final {
            bindInterface(static_cast<CommandAdapter*>(Adapter));
            return 0;
        }

        void bindInterface(CommandAdapter* adapter) {
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