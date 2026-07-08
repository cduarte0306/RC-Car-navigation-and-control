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
#include <thread>
#include <condition_variable>

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
        int bindCommandDispatch(std::function<int(Msg::MessageCapsule<std::vector<char>>& capsule)> dispatchFunc);

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
        virtual int dispatchCommand(Msg::MessageCapsule<std::vector<char>>& capsule);

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
         * @brief Get the parent name
         * 
         * @return std::string parent module name 
         */
        std::string getParentName() const;
        
        /**
         * @brief Get the device type associated with this adapter
         * 
         * @return ModuleDefs::AdapterId The adapter's device type identifier
         */
        const ModuleDefs::AdapterId getDeviceType() const {
            return adapterId;
        }

        /**
         * @brief Acknowledge a received message by sending a reply back to the adapter. This can be used by command 
         * handlers to provide any necessary response data back to the adapter after processing a command.
         * 
         * @param capsule The message capsule containing the original command and data
         * @param reply Buffer containing any response data to be sent back to the adapter
         * @param len Length of the reply buffer
         * @return int Error code indicating success or failure of the acknowledgment process
         */
        virtual int AckMsg(Msg::MessageCapsule<std::vector<char>>& capsule, char* reply, int len);

        /**
         * @brief Get the parent module's ID
         * 
         * @return int Parent module ID, or -1 if not found
         */
        int GetParentID() const {
            return m_ModuleID;
        }

        /**
         * @brief Set the Mod Dispatch Callback for adapter-module comms
         * 
         * @param modDispatchCB Callback for module command dispatching
         * @return int 
         */
        int SetModDispatchCallback(std::function<int(Msg::MessageCapsule<std::vector<char>>&)> modDispatchCB);

        /**
         * @brief Set the reply handler callback for processing module acknowledgments
         * 
         * @param replyHandlerCB Callback for handling module acknowledgments
         * @return int 
         */
        int SetReplyHandlerCallback(std::function<int(Msg::MessageAck<std::vector<char>>&)> replyHandlerCB);

        /**
         * @brief Forward a module acknowledgment through the bound reply callback chain.
         *
         * This is typically called by module-side code to route a reply back to
         * the adapter that originated the command.
         */
        int ConnectModuleReply(Msg::MessageAck<std::vector<char>>& ack);
    protected:
        std::thread m_ReplyProcThread;
        std::thread m_InputProcThread;
        std::string parentName;
        std::unordered_map<std::string, AdapterBase*> adapterMap;
        std::function< int(char* pbuf, size_t len) > moduleWriteCmd = nullptr;
        std::function< int(std::vector<std::string>&) > moduleCliCmd = nullptr;
        std::function< int(std::vector<char>&)     > moduleWriteCmdVector = nullptr;
        std::function<std::string(void)> readStatsCommand = nullptr;
        std::function<int(Msg::MessageCapsule<std::vector<char>>&)> dispatchCommandFunc = nullptr;
        std::function<int(std::vector<char>& buffer)> OnModuleMsgReceivedFunc = nullptr;
        std::function<int(Msg::MessageCapsule<std::vector<char>>&)> moduleDispatchCmd = nullptr;
        std::function<int(Msg::MessageAck<std::vector<char>>&)> moduleReplyCmd = nullptr;
        std::function<int(Msg::MessageAck<std::vector<char>>&)> replyHandlerFunc = nullptr;
        Msg::CircularBuffer<Msg::MessageAck<std::vector<char>>> m_ReplyMailBox;
        Msg::CircularBuffer<Msg::MessageCapsule<std::vector<char>>> m_InputMailBox;
        
        int m_ModuleID = -1;
        bool m_ReplyThreadRunning{true};

        const ModuleDefs::AdapterId adapterId;

        virtual int moduleCommand_(char* pbuf, size_t len);

        virtual int moduleCommand_(std::vector<char>& buffer);

        virtual int moduleCliCmd_(std::vector<std::string>& buffer);

        void procReplyThread(void);

        void procInputThread(void);

        /**
         * @brief Submit a message capsule to the adapter's input mailbox for processing by the module. This can be used by command handlers or other adapter logic to forward messages to the module for handling.
         * 
         * @param capsule The message capsule containing the command and data to be submitted to the module
         * @return int Error code indicating success or failure of the submission process
         */
        int SubmitMailBox(Msg::MessageCapsule<std::vector<char>>& capsule) {
            if (m_InputMailBox.isFull()) {
                return -1;
            }

            m_InputMailBox.push(capsule);
            return 0; 
        }

        /**
         * @brief Submit an acknowledgment to the adapter's reply mailbox for processing by the reply thread. This can be used by command handlers to send acknowledgments back to the adapter, which can then be forwarded to the appropriate destination (e.g. network adapter) for transmission back to the original sender.
         * 
         * @param ack The acknowledgment object containing the acknowledgment data to be sent back to the sender module thread
         * @return int Error code indicating success or failure of the submission process
         */
        int SubmitReplyMailBox(Msg::MessageAck<std::vector<char>>& ack) {
            if (m_ReplyMailBox.isFull()) {
                return -1;
            }

            m_ReplyMailBox.push(ack);
            return 0; 
        }
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
        std::function<int(bool)>     setStreamingStateCommand = nullptr;

        virtual int bind_(AdapterBase* Adapter) override;

        void bindInterface(CLIAdapter* adapter);

        virtual std::string readModuleStats_(void);
    };

    class CameraAdapter : public AdapterBase {
    public:
        CameraAdapter(std::string parentName_="");

        int setCameraState(bool state);

        int StartStreaming();

        int StopStreaming();

        int SetStreamingState(bool state);

        /**
         * @brief Read stats from module
         * 
         * @return std::string 
         */
        virtual std::string readStats() override;

    protected:
        std::function<int(bool)>                    setCameraStateCommand    = nullptr;
        std::function<int(void)>                    startStreamingCommand    = nullptr;
        std::function<int(void)>                    stopStreamingCommand     = nullptr;

        virtual int bind_(AdapterBase* Adapter) override;

        void bindInterface(CameraAdapter* adapter);

        virtual int startStreaming_();

        virtual int stopStreaming_();

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

        class NetworkAdapter {
        public:
            NetworkAdapter(const std::string& adapter_, int sPort_, int dPort_, size_t bufferSize_=2048);
            ~NetworkAdapter();
            std::function<int(const uint8_t*, size_t)> sendCallbacEth = nullptr;
            std::function<int(const uint8_t*, size_t)> sendCallbackWlan = nullptr;
            std::function<int(const uint8_t*, size_t)> sendCallback = nullptr;
            std::function<int(const uint8_t*, size_t)> sendCallbackTcp = nullptr;
            std::function<bool(void)>                  EthPresent = nullptr;
            std::function<bool(void)>                  hostPresentCB = nullptr;
            std::function<void()> onConnected = nullptr;

            int id = -1;
            int typeID = -1;
            int sPort = -1;
            int dPort = -1;
            const size_t bufferSize = 0;
            bool connected = false;
            std::string adapter;
            std::string parent;
            std::atomic<bool> wlanLinkDetected;
            std::atomic<bool> ethLinkDetected;
            bool broadcast = false;
            int adapterType = -1;

            int socketDesc{-1};

            int send(const uint8_t* data, size_t length, std::string destIp="");

            bool IsEthPresent(void) const;

            bool IsHostPresent(void) const;

            void setParent(const std::string& name);
            
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