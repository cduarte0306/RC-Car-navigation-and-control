#pragma once

#include <map>
#include <string>
#include <unordered_map>
#include <list>
#include <utility>
#include <atomic>
#include <functional>
#include <iostream>
#include <memory>

#include "ModulesDefs.hpp"

#include "lib/MessageLib.hpp"

namespace Adapter {
    class AdapterBase {
    public:    
        AdapterBase(ModuleDefs::AdapterId id, std::string parentName_="");
        virtual ~AdapterBase();

        /**
         * @brief Bind this adapter to a module adapter. This allows the module and adapter to communicate by forwarding calls between them. The module will call the adapter's moduleCommand, which will forward to the bound module adapter's implementation, and the module adapter can call this adapter's moduleCommand to forward commands back to the module.
         * 
         * @param Adapter Pointer to the adapter to bind
         * @return int Status code (0 for success, -1 for failure)
         */
        int bind(AdapterBase* Adapter);

        /**
         * @brief Bind a command dispatch function to this adapter. This allows the adapter to forward received commands to the module's dispatchCommand implementation.
         * 
         * @param dispatchFunc The dispatch function to bind, which should match the signature of dispatchCommand (taking a MessageCapsule and returning an int)
         * @return int Status code (0 for success, -1 for failure)
         */
        int bindCommandDispatch(std::function<int(Msg::MessageCapsule<char>& capsule)> dispatchFunc);

        /**
         * @brief Bind module message reception callback. This allows the adapter to forward 
         * received messages to the module's OnModuleMsgReceived implementation.
         * 
         * @return std::string Stats information as a string
         */
        int bindOnModuleMsgReceived(std::function<int(std::vector<char>& buffer)> onMsgReceivedFunc);

        /**
         * @brief Default implementation of readStats. Adapters that need to provide custom handling should override this.
         * 
         * @return std::string Stats information as a string
         */
        virtual std::string readStats();

        /**
         * @brief Bind an adapter to this adapter. Must be implemented by derived classes.
         * 
         * @param Adapter Pointer to the adapter to bind
         * @return int Status code
         */
        virtual int bind_(AdapterBase* Adapter)= 0;

        /**
         * @brief Default implementation of moduleCommand. Adapters that need to
         * provide custom handling should override this.
         */
        virtual int stopCmd(void);

        /**
         * @brief Default implementation of moduleCommand. Modules or adapters that need to
         * provide custom handling should override this.
         */
        virtual int dispatchCommand(Msg::MessageCapsule<char>& capsule);

        /**
         * Default handler for moduleCommand. Modules or adapters that need to
         * provide custom handling should override this. Providing a non-pure
         * implementation here allows concrete adapter types to be instantiated
         * without requiring every adapter to implement moduleCommand.
         */
        virtual int moduleCommand(char* pbuf, size_t len);

        /**
         * @brief Default handler for moduleCommand with vector input. Modules or adapters that need to
         * provide custom handling should override this.
         */
        virtual int moduleCommand(std::vector<char>& buffer);

        virtual int cliCommand(std::vector<std::string>& buffer);

        /**
         * @brief Add a module name to the bound modules list
         * 
         * @param moduleName Name of the module to add
         */
        void addAdapter(std::string moduleName);

        /**
         * @brief Get the parent name
         * 
         * @return std::string parent module name 
         */
        std::string getParentName() const;

        /**
         * @brief Get the parent module's ID
         * 
         * @return int Parent module ID, or -1 if not found
         */
        int GetParentID() const {
            return m_ModuleID;
        }
    protected:
        std::string parentName;
        std::list<std::string> boundModules;
        std::unordered_map<std::string, AdapterBase*> adapterMap;
        std::function< int(char* pbuf, size_t len) > moduleWriteCmd = nullptr;
        std::function< int(std::vector<std::string>&) > moduleCliCmd = nullptr;
        std::function< int(std::vector<char>&)     > moduleWriteCmdVector = nullptr;
        std::function< int(char* pbuf, size_t len) > moduleWriteAsyncCmd = nullptr;
        std::function<std::string(void)> readStatsCommand = nullptr;
        std::function<int(Msg::MessageCapsule<char>& capsule)> dispatchCommandFunc = nullptr;
        std::function<int(std::vector<char>& buffer)> OnModuleMsgReceivedFunc = nullptr;
        int m_ModuleID = -1;

        const ModuleDefs::AdapterId adapterId;

        virtual int moduleCommand_(char* pbuf, size_t len);

        virtual int moduleCommand_(std::vector<char>& buffer);

        virtual int moduleCommandAsync_(char* pbuf, size_t len);

        virtual int moduleCliCmd_(std::vector<std::string>& buffer);
    };

    class MotorAdapter : public AdapterBase {
    public:
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
    
    class CLIAdapter : public AdapterBase {
    public:
        CLIAdapter(std::string parentName_="");

    protected:
        std::function<std::string()> readStats = nullptr;

        virtual int bind_(AdapterBase* Adapter) override;

        void bindInterface(CLIAdapter* adapter);

        virtual std::string readModuleStats_(void);
    };

    class CameraAdapter : public AdapterBase {
    public:
        CameraAdapter(std::string parentName_="");

        int setCameraState(bool state);

        /**
         * @brief Read stats from module
         * 
         * @return std::string 
         */
        virtual std::string readStats() override;

    protected:
        std::function<int(bool)>                    setCameraStateCommand    = nullptr;

        virtual int bind_(AdapterBase* Adapter) override;

        void bindInterface(CameraAdapter* adapter);

        virtual int setCameraState_(int direction);

        virtual int configurePipeline_(const std::string& host);
    };

    class UpdateAdapter : public AdapterBase {
    public:
        UpdateAdapter(std::string parentName_="");
    protected:
        virtual int bind_(AdapterBase* Adapter) override;

        void bindInterface(UpdateAdapter* adapter);
    };

    class CommsAdapter : public AdapterBase {
    public:
        enum {
            MaxUDPPacketSize = 65507
        };

        enum {
            UdpAdapterType = 1,
            TcpAdapterType = 2
        };

        struct NetworkAdapter {
            NetworkAdapter(const std::string& adapter_,  int sPort_, int dPort_, size_t bufferSize_=2048);
            ~NetworkAdapter();
            std::function<int(std::string, const uint8_t*, size_t)> sendCallback = nullptr;
            std::function<int(const uint8_t*, size_t)> sendCallbackTcp = nullptr;
            std::function<void(void)> OnEthDetected = nullptr;
            std::function<void()> onConnected = nullptr;
            std::function<std::string()> hostResolver = nullptr;
            int id = -1;
            int typeID = -1;
            std::string adapter;
            int sPort = -1;
            int dPort = -1;
            const size_t bufferSize = 0;
            bool connected = false;
            std::string parent;
            std::atomic<bool> wlanLinkDetected;
            std::atomic<bool> ethLinkDetected;
            bool broadcast = false;
            int adapterType = -1;

            int send(const uint8_t* data, size_t length, std::string destIp="");

            void setParent(const std::string& name);

            std::string getHostIP() const;
            
            void OnEthLinkDetected(bool state);

            void OnWlanLinkDetected(bool state);

        };

        CommsAdapter(std::string parentName_="");


        /**
         * @brief Transmit data through the communication adapter
         * @param data Pointer to data buffer
         * @param length Size of data
         * @return int Status code  
         */
        virtual int transmitData(const uint8_t* data, size_t length);


        /**
         * @brief Start receiving data asynchronously
         * 
         * @param callback Callback function to handle received data
         */
        virtual int startReceive(NetworkAdapter& adapter, std::function<void(std::vector<char>&)> callback, bool asyncTx=true);

        /**
         * @brief Start receiving data asynchronously for module message dispatch mode. All network receptiobs
         * are routed to the module OnMsgReceived Callback
         * 
         * @param callback Callback function to handle received data
         */
        virtual int startReceive(NetworkAdapter& adapter);

        /**
         * @brief Get the host IP address associated with the adapter
         * 
         * @param adapter Reference to the network adapter
         * @return std::string Host IP address as a string
         */
        virtual std::string getHostIP(NetworkAdapter& adapter);

        /**
         * @brief Open an adapter for a given parent module
         * 
         * @param port Port number for the adapter
         * @param adapter Adapter identifier or name
         * @return int 
         */
        virtual int openAdapter(int port, std::string& adapter);

        virtual std::unique_ptr<NetworkAdapter> createNetworkAdapter(const std::string& callerName, uint8_t type, int sPort, int dPort, std::string adapter, size_t bufferSize=2048, bool broadcast=false);

        /**
         * @brief Read stats from module
         * 
         * @return std::string 
         */
        virtual std::string readStats() override;
        
        /**
         * @brief Get the Eth Connection State 
         * 
         * @return true 
         * @return false 
         */
        virtual bool GetEthConnectionState() const;
    protected:
        // callable to request data transmit; now includes caller identity
        std::function<int(const uint8_t*, size_t)                                                       > transmitDataCommand   = nullptr;
        std::function<std::unique_ptr<NetworkAdapter>(const std::string&, int, int, const std::string&, size_t, bool)> openAdapterCommand    = nullptr;
        std::function<std::unique_ptr<NetworkAdapter>(const std::string&, int, int, const std::string&, size_t, bool)> openTcpAdapterCommand = nullptr;

        std::function<int (const char* pbuf, size_t len)                                                > recvDataCallback      = nullptr;
        std::function<int(NetworkAdapter& adapter, std::function<void(std::vector<char>&)>, bool)> dataReceivedCommand = nullptr;
        std::function<std::string(NetworkAdapter& adapter)> hostIPQueryCommand = nullptr;
        std::atomic<bool> ethConnectionState{false};
        int adapterCounter = -1;

        std::list<std::pair<std::string, std::string>> m_RegisteredCallers;  // List of modules that have opened an adapter here
        // Fast lookup from caller module name -> adapter pointer (populated on open)
        std::unordered_map<std::string, CommsAdapter*> m_CallerAdapterMap;

        /**
         * @brief Bind interface for communication adapter
         * 
         * @param Adapter Pointer to communication adapter
         */
        virtual int bind_(AdapterBase* Adapter) final;


        /**
         * @brief Bind interface for communication adapter
         * 
         * @param adapter Pointer to communication adapter
         */
        void bindInterface(CommsAdapter* adapter);


        /**
         * @brief Start receiving data asynchronously
         * 
         * @param dataReceivedCommand_ Callback function to handle received data
         */
        virtual void startReceive_(NetworkAdapter& adapter, std::function<void(std::vector<char>&)> dataReceivedCommand_, bool asyncTx=true);

        virtual void configureReceiveCallback(NetworkAdapter& adapter, std::function<void(std::vector<char>&)> callback, bool asyncTx=true);

        virtual std::string getHostIP_(NetworkAdapter& adapter);


        /**
         * @brief Transmit data on behalf of caller
         * 
         * @param data Pointer to the data to be transmitted
         * @param length Length of the data to be transmitted
         * @return int Status code of the transmission operation
         */
        virtual int transmitData_(const uint8_t* data, size_t length);


        /**
         * @brief Open an adapter for a given parent module
         * 
         * @param parent Identifier of the parent module requesting the adapter
         * @param sPort Source port number for the adapter
         * @param dPort Destination port number for the adapter
         * @param adapter Adapter identifier or name
         * @return int Status code of the operation
         */
        virtual std::unique_ptr<NetworkAdapter> openAdapter_(const std::string& parent,  int sPort, int dPort, const std::string& adapter, size_t bufferSize, bool broadcast) final;


        /**
         * @brief Open a TCP adapter for a given parent module
         * 
         * @param parent Identifier of the parent module requesting the adapter
         * @param sPort Source port number for the adapter
         * @param dPort Destination port number for the adapter
         * @param adapter Adapter identifier or name
         * @return int Status code of the operation
         */
        virtual std::unique_ptr<NetworkAdapter> openTcpAdapter_(const std::string& parent,  int sPort, int dPort, const std::string& adapter, size_t bufferSize, bool broadcast);

        
        /**
         * @brief Configure an adapter for a given port
         * 
         * @param port Port number for the adapter
         * @param adapter Adapter identifier or name
         * @return int Status code of the operation
         */
        virtual int configureUDPAdapter(NetworkAdapter& netAdapter, int adapterIdx);

        /**
         * @brief Configure a TCP adapter for a given port
         * 
         * @param port Port number for the adapter
         * @param adapter Adapter identifier or name
         * @return int Status code of the operation
         */
        virtual int configureTCPAdapter(NetworkAdapter& netAdapter, int adapterIdx);
    };

    class CommandAdapter : public AdapterBase {
    public:
        CommandAdapter(std::string parentName_="");

    protected:
        // callable to request motor speed; empty when not set

        virtual int bind_(AdapterBase* Adapter) final;

        void bindInterface(CommandAdapter* adapter);
    };

    class TlmAdapter : public AdapterBase {
    public:
        TlmAdapter(std::string parentName_="");

        int registerTelemetrySource(const std::string& sourceName);

        int publishTelemetry(const std::string& sourceName, const uint8_t* data, size_t length);

        int publishTelemetry(const std::string& sourceName, const std::string& payload);
        
    protected:
        std::function<int(const std::string&)> registerSourceCommand = nullptr;
        std::function<int(const std::string&, const uint8_t*, size_t)> publishTelemetryCommand = nullptr;

        virtual int bind_(AdapterBase* Adapter) final;

        void bindInterface(TlmAdapter* adapter);

        virtual int registerTelemetrySource_(const std::string& sourceName);

        virtual int publishTelemetry_(const std::string& sourceName, const uint8_t* data, size_t length);
    };
}

#pragma endregion