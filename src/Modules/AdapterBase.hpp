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
        CameraAdapterID,
        TlmAdapterID
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

        virtual int moduleCommand(std::vector<char>& buffer) {
            if (!moduleWriteCmd) {
                std::cout << "Module command error\r\n";
                return -1;
            }

            return moduleWriteCmdVector(buffer);
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
        std::function< int(std::vector<char>&)     > moduleWriteCmdVector = nullptr;
        std::function< int(char* pbuf, size_t len) > moduleWriteAsyncCmd = nullptr;

        const int adapterId;

        virtual int moduleCommand_(char* pbuf, size_t len) {
            (void)pbuf; (void)len;
            return -1;
        }

        virtual int moduleCommand_(std::vector<char>& buffer) {
            (void)buffer;
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
        CameraAdapter(std::string parentName_="") : AdapterBase(CameraAdapterID, parentName_){}

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

            this->moduleWriteCmd = [adapter](char* pbuf, size_t len) {
                return adapter->moduleCommand_(pbuf, len);
            };

            this->moduleWriteCmdVector = [adapter](std::vector<char>& buffer) {
                return adapter->moduleCommand_(buffer);
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
        enum {
            MaxUDPPacketSize = 65507
        };

        struct NetworkAdapter {
            NetworkAdapter(std::string& adapter_, int port_, size_t bufferSize_=2048) : adapter(adapter_), port(port_), bufferSize(bufferSize_) {}
            ~NetworkAdapter() {}
            std::function<int(std::string, const uint8_t*, size_t)> sendCallback = nullptr;
            std::function<std::string()> hostResolver = nullptr;
            int id = -1;
            std::string adapter;
            const int port = -1;
            const size_t bufferSize = 0;
            bool connected = false;

            int send(std::string destIp, const uint8_t* data, size_t length) {
                if (sendCallback) {
                    return sendCallback(destIp, data, length);
                }
                return -1;
            }

            std::string getHostIP() const {
                if (hostResolver) {
                    return hostResolver();
                }
                return std::string();
            }

        };

        CommsAdapter(std::string parentName_="") : AdapterBase(CommsAdapterID, parentName_) {}


        /**
         * @brief Transmit data through the communication adapter
         * @param data Pointer to data buffer
         * @param length Size of data
         * @return int Status code  
         */
        virtual int transmitData(const uint8_t* data, size_t length) {
            if (!data || length == 0) {
                return -1;
            }

            return transmitDataCommand(data, length);
        }


        /**
         * @brief Start receiving data asynchronously
         * 
         * @param callback Callback function to handle received data
         */
        virtual int startReceive(NetworkAdapter& adapter, std::function<void(std::vector<char>&)> callback, bool asyncTx=true) {
            if (!callback) {
                return -1;
            }

            dataReceivedCommand(adapter, callback, asyncTx);
            return 0;
        }

        virtual std::string getHostIP(NetworkAdapter& adapter) {
            if (!hostIPQueryCommand) {
                return std::string();
            }
            return hostIPQueryCommand(adapter);
        }

        /**
         * @brief Open an adapter for a given parent module
         * 
         * @param port Port number for the adapter
         * @param adapter Adapter identifier or name
         * @return int 
         */
        virtual int openAdapter(int port, std::string& adapter) {
            return 0;
        }

        virtual std::unique_ptr<NetworkAdapter> createNetworkAdapter(int port, std::string adapter, size_t bufferSize=2048) {
            return openAdapterCommand(parentName, port, adapter, bufferSize);  // Pass the source's name to the sink adapter's openAdapter
        }
    protected:
        // callable to request data transmit; now includes caller identity
        std::function<int(const uint8_t*, size_t)                                                       > transmitDataCommand   = nullptr;
        std::function<std::unique_ptr<NetworkAdapter>(std::string&, int, std::string&, size_t)          > openAdapterCommand    = nullptr;
        std::function<int (const char* pbuf, size_t len)                                                > recvDataCallback      = nullptr;
        std::function<int(NetworkAdapter& adapter, std::function<void(std::vector<char>&)>, bool)> dataReceivedCommand = nullptr;
        std::function<std::string(NetworkAdapter& adapter)> hostIPQueryCommand = nullptr;
        int adapterCounter = -1;

        std::list<std::pair<std::string, std::string>> m_RegisteredCallers;  // List of modules that have opened an adapter here
        // Fast lookup from caller module name -> adapter pointer (populated on open)
        std::unordered_map<std::string, AdapterBase*> m_CallerAdapterMap;

        /**
         * @brief Bind interface for communication adapter
         * 
         * @param Adapter Pointer to communication adapter
         */
        virtual int bind_(AdapterBase* Adapter) final {
            bindInterface(static_cast<CommsAdapter*>(Adapter));
            return 0;
        }


        /**
         * @brief Bind interface for communication adapter
         * 
         * @param adapter Pointer to communication adapter
         */
        void bindInterface(CommsAdapter* adapter) {            
            if (!adapter) return;

            // Forward calls to the bound concrete comms adapter (e.g., NetworkComms)
            this->transmitDataCommand = [adapter](const uint8_t* pData, size_t length) -> int {
                return adapter->transmitData_(pData, length);
            };

            this->openAdapterCommand = [adapter](std::string& parent, int port, std::string& adpName, size_t bufferSize) -> std::unique_ptr<NetworkAdapter> {
                return adapter->openAdapter_(parent, port, adpName, bufferSize);
            };
            
            this->dataReceivedCommand = [adapter](NetworkAdapter& netAdp, std::function<void(std::vector<char>&)> callback, bool asyncTx) -> int {
                adapter->configureReceiveCallback(netAdp, callback, asyncTx);
                return 0;
            };

            this->hostIPQueryCommand = [adapter](NetworkAdapter& netAdp) -> std::string {
                return adapter->getHostIP_(netAdp);
            };
        }


        /**
         * @brief Start receiving data asynchronously
         * 
         * @param dataReceivedCommand_ Callback function to handle received data
         */
        virtual void startReceive_(NetworkAdapter& adapter, std::function<void(std::vector<char>&)> dataReceivedCommand_, bool asyncTx=true) {
            // Default implementation doesn't know how to route - subclasses
            // (e.g. a network comms controller) should override this method
            // and use `caller` to pick the fastest receive path.
        }


        virtual void configureReceiveCallback(NetworkAdapter& adapter, std::function<void(std::vector<char>&)> callback, bool asyncTx=true) {
            
        }

        virtual std::string getHostIP_(NetworkAdapter& adapter) {
            return adapter.getHostIP();
        }


        /**
         * @brief Transmit data on behalf of caller
         * 
         * @param data Pointer to the data to be transmitted
         * @param length Length of the data to be transmitted
         * @return int Status code of the transmission operation
         */
        virtual int transmitData_(const uint8_t* data, size_t length) {
            if (!data || length == 0) {
                return -1;
            }

            // Default implementation doesn't know how to route - subclasses
            // (e.g. a network comms controller) should override this method
            // and use `caller` to pick the fastest transmit path.
            return 0;
        }


        /**
         * @brief Open an adapter for a given parent module
         * 
         * @param parent Identifier of the parent module requesting the adapter
         * @param port Port number for the adapter
         * @param adapter Adapter identifier or name
         * @return int Status code of the operation
         */
        virtual std::unique_ptr<NetworkAdapter> openAdapter_(std::string& parent, int port, std::string& adapter, size_t bufferSize) final {
            std::pair<std::string, std::string> adapterDesc(parent, adapter);
            m_RegisteredCallers.push_back(adapterDesc);

            // If we have previously bound this caller adapter into adapterMap
            // (via AdapterBase::bind), cache a direct pointer for fast lookup
            auto it = adapterMap.find(parent);
            if (it != adapterMap.end()) {
                m_CallerAdapterMap[parent] = it->second;
            }

            std::unique_ptr<NetworkAdapter> netAdapter = std::make_unique<NetworkAdapter>(adapter, port, bufferSize);
            adapterCounter++;
            netAdapter->id = adapterCounter;  // assign unique ID

            // Configure the adapter on the derived comms driver
            const int cfgStatus = configureAdapter(*netAdapter, netAdapter->id);
            if (cfgStatus != 0) {
                std::string msg("Failed to configure adapter " + adapter + " for parent " + parent + "\n");
            }

            return netAdapter;
        }

        
        /**
         * @brief Configure an adapter for a given port
         * 
         * @param port Port number for the adapter
         * @param adapter Adapter identifier or name
         * @return int Status code of the operation
         */
        virtual int configureAdapter(NetworkAdapter& netAdapter, int adapterIdx) {
            return 0;
        }
    };

    class CommandAdapter : public AdapterBase {
    public:
        CommandAdapter(std::string parentName_="") : AdapterBase(CommandAdapterID, parentName_) {}

    protected:
        // callable to request motor speed; empty when not set

        virtual int bind_(AdapterBase* Adapter) final {
            bindInterface(static_cast<CommandAdapter*>(Adapter));
            return 0;
        }

        void bindInterface(CommandAdapter* adapter) {
            if (!adapter) return;
        }
    };

    class TlmAdapter : public AdapterBase {
    public:
        TlmAdapter(std::string parentName_="") : AdapterBase(TlmAdapterID, parentName_) {}

        int registerTelemetrySource(const std::string& sourceName) {
            if (!registerSourceCommand) {
                return -1;
            }

            return registerSourceCommand(sourceName);
        }

        int publishTelemetry(const std::string& sourceName, const uint8_t* data, size_t length) {
            if (!publishTelemetryCommand || !data || length == 0) {
                return -1;
            }

            return publishTelemetryCommand(sourceName, data, length);
        }

        int publishTelemetry(const std::string& sourceName, const std::string& payload) {
            return publishTelemetry(sourceName, reinterpret_cast<const uint8_t*>(payload.data()), payload.size());
        }
        
    protected:
        std::function<int(const std::string&)> registerSourceCommand = nullptr;
        std::function<int(const std::string&, const uint8_t*, size_t)> publishTelemetryCommand = nullptr;

        virtual int bind_(AdapterBase* Adapter) final {
            bindInterface(static_cast<TlmAdapter*>(Adapter));
            return 0;
        }

        void bindInterface(TlmAdapter* adapter) {
            if (!adapter) return;

            registerSourceCommand = [adapter](const std::string& sourceName) -> int {
                return adapter->registerTelemetrySource_(sourceName);
            };

            publishTelemetryCommand = [adapter](const std::string& sourceName, const uint8_t* data, size_t length) -> int {
                return adapter->publishTelemetry_(sourceName, data, length);
            };

            this->moduleWriteCmd = [adapter](char* pbuf, size_t len) {
                return adapter->moduleCommand_(pbuf, len);
            };

            this->moduleWriteAsyncCmd = [adapter](char* pbuf, size_t len) {
                return adapter->moduleCommandAsync_(pbuf, len);
            };
        }

        virtual int registerTelemetrySource_(const std::string& sourceName) {
            (void)sourceName;
            return 0;
        }

        virtual int publishTelemetry_(const std::string& sourceName, const uint8_t* data, size_t length) {
            (void)sourceName;
            (void)data;
            (void)length;
            return -1;
        }
    };
}

#pragma endregion