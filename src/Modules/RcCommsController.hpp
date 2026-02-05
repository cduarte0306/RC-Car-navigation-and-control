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

    // Override moduleCommand to handle incoming commands
    virtual int moduleCommand(char* pbuf, size_t len) override {
        return 0;
    }

    virtual void OnTimer(void);

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

    // CLI spefic stats reading
    virtual std::string readStats() override;

    // WLAN write function
    int wlanWrite(const uint8_t* data, size_t length);

    // Ethernet write function
    int ethWrite(const uint8_t* data, size_t length);

protected:
    enum {
        EthAdapter,
        WlanAdapter,
        MaxAdapter
    };

    struct NetStats {
        int port;
        uint64_t txRate;
        uint64_t rxRate;
        std::string moduleName;
        std::unique_ptr<Network::UdpServer> socket;
        Adapter::CommsAdapter::NetworkAdapter* netAdapter = nullptr;
    };
    
    // Map of UDP sockets by adapter ID
    std::unordered_map<int, NetStats> m_UdpSockets;

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