#include <thread>
#include <mutex>
#include <atomic>
#include <types.h>
#include <map>
#include "RcBase.hpp"

#include "Devices/network_interface/UdpServer.hpp"

using namespace std;
using namespace Adapter;

namespace Modules {
class NetworkComms : public Base, public Adapter::CommsAdapter {
public:
    NetworkComms(int moduleID, std::string name);
    ~NetworkComms();

    virtual int init(void) override {
        // Implementation to initialize the motor controller
        return 0;
    }

    virtual int stop(void) override {
        // Implementation to stop the motor controller
        return 0;
    }

    Adapter::AdapterBase* getInputAdapter() override {
        return static_cast<Adapter::AdapterBase*>(static_cast<Adapter::CommsAdapter*>(this));
    }
protected:
    enum {
        CMD_NOOP,
        CMD_FWD_DIR,
        CMD_STEER,
    };

    typedef struct __attribute__((__packed__))
    {
        uint32_t   sequence_id;
        uint16_t   msg_length;

        struct __attribute__((__packed__))
        {
            uint8_t    command;
            val_type_t data;
        } payload;
    } ClientReq_t;

    typedef struct __attribute__((__packed__)) {
        val_type_t data;
        uint8_t state;    
    } reply_t;

    // Override moduleCommand to handle incoming commands
    virtual int moduleCommand(char* pbuf, size_t len) override {
        return 0;
    }

    // Main Process
    virtual void mainProc();

    // Opens network adapters
    virtual int configureAdapter(NetworkAdapter& netAdapter, int adapterIdx) override;

    // Override startReceive_ to route incoming data via UDP
    virtual void configureReceiveCallback(NetworkAdapter& adapter, std::function<void(std::vector<char>&)> dataReceivedCommand_, bool asyncTx=true) override;

    // Override transmitData_ to route data via UDP
    virtual int transmitData_(const uint8_t* data, size_t length) override;

    // Provide host IP lookup for bound adapters
    virtual std::string getHostIP_(NetworkAdapter& adapter) override;

    // WLAN write function
    int wlanWrite(const uint8_t* data, size_t length);

    // Ethernet write function
    int ethWrite(const uint8_t* data, size_t length);
    
    // Map of UDP sockets by adapter ID
    std::unordered_map<int, std::unique_ptr<Network::UdpServer>> m_UdpSockets;

    // Map of adapter name to non-owning socket pointers
    std::unordered_map<std::string, Network::UdpServer*> m_AdapterMap;

    // Ethernet adapter known host
    std::string m_EthHostIP;  // Ethernet known host

    // WLAN known host
    std::string m_WlanHostIP; // WLAN known host

    // Non-owning pointer to the primary socket (first configured adapter)
    Network::UdpServer* m_UdpSocket{nullptr};

    // List of adapter names that failed to open
    std::vector<std::pair<int, Adapter::CommsAdapter::NetworkAdapter*>> m_FailedAdapters;

    // Map of adapter IDs to failed adapter structs for quick lookup
    std::map<int, Adapter::CommsAdapter::NetworkAdapter*> m_FailedAdapterMap;
};
};