#pragma once
#ifndef ADAPTERCOMMS_HPP
#define ADAPTERCOMMS_HPP

#include "AdapterBase.hpp"

namespace Adapter {
    class CommsAdapter : public AdapterBase {
    public:
        enum {
            MaxUDPPacketSize = 65507
        };

        enum {
            Udp       = 1,
            TcpServer = 2,
            TcpClient = 3,
            TcpProxy  = 4
        };

        enum {
            UdpAdapterType       = 1,
            TcpServerAdapterType = 2,
            TcpClientAdapterType = 3,
            TcpProxyAdapterType  = 4
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

        /**
         * @brief Create a remote network adapter
         * 
         * @param callerName Name of the caller module
         * @param sPort Source port
         * @param dPort Destination port
         * @param adapter Adapter identifier or name
         * @param bufferSize Size of the buffer
         * @param broadcast Whether to enable broadcast
         * @return std::unique_ptr<NetworkAdapter> 
         */
        template<typename T, int BufferSize=2048>
        std::unique_ptr<T> OpenNetworkAdapter(const std::string& callerName, int sPort, int dPort, bool internal=false, bool broadcast=false);

        /**
         * @brief Read stats from module
         * 
         * @return std::string 
         */
        virtual std::string readStats() override;

        /**
         * @brief Refresh connection to host
         * 
         */
        virtual void RefreshConnectionState();

        /**
         * @brief Get the Eth Connection State 
         * 
         * @return true 
         * @return false 
         */
        virtual bool GetEthConnectionState() const;
    protected:
        // callable to request data transmit; now includes caller identity
        std::function<int(const uint8_t*, size_t)>                                                             transmitDataCommand    = nullptr;
        std::function<std::unique_ptr<NetworkAdapter>(const std::string&, int, int, int, size_t, bool, bool)>  openAdapterCommand     = nullptr;
        std::function<int(NetworkAdapter& adapter, std::function<void(std::vector<char>&)>, bool)>             dataReceivedCommand    = nullptr;
        std::function<int(NetworkAdapter& adapter, std::function<void(std::string&)>)>                         clientConnectedCommand = nullptr;
        std::function<void(void)>                                                                              refreshConnectionStateCommand = nullptr;
        std::function<std::unique_ptr<NetworkProxy>(const std::string&, int, int, std::string, size_t, bool)>  openProxyCommand       = nullptr;
        
        std::function<std::string(NetworkAdapter& adapter)> hostIPQueryCommand = nullptr;
        std::atomic<bool> ethConnectionState{false};
        bool mIsProxy{false};
        int adapterCounter = -1;

        std::list<std::string> m_RegisteredCallers;  // List of modules that have opened an adapter here
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
         * @brief Configure the tcp on connect callback
         * 
         * @param adapter 
         * @param callback 
         */
        virtual void configureOnConnectCallback(NetworkAdapter& adapter, std::function<void(std::string& hostIp)> callback);

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
        virtual std::unique_ptr<NetworkAdapter> openAdapter_(const std::string& parent, int type, int sPort, int dPort, size_t bufferSize, bool broadcast, bool loopback) final;

        /**
         * @brief Configure an adapter for a given port
         * 
         * @param port Port number for the adapter
         * @param adapter Adapter identifier or name
         * @param type Type of the adapter (e.g., UDP, TCP)
         * @param remote Flag indicating whether the adapter is remote or local
         * @return int Status code of the operation
         */
        virtual int configureAdapter(NetworkAdapter& netAdapter, int adapterIdx, int type, bool internal=false);

        /**
         * @brief Refresh connection to host
         * 
         */
        virtual void RefreshConnectionState_();
    };
}

#endif // ADAPTERCOMMS_HPP
