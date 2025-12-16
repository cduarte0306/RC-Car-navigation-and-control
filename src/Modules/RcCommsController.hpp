#include <thread>
#include <mutex>
#include <atomic>
#include <types.h>
#include "RcBase.hpp"

#include "Devices/network_interface/UdpServer.hpp"

using namespace std;
using namespace Adapter;

namespace Modules {
class NetworkComms : public Base, public Adapter::CommsAdapter {
public:
    NetworkComms(int moduleID, string name);
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

    // Map of UDP sockets by adapter ID
    unordered_map<int, unique_ptr<Network::UdpServer>> m_UdpSockets;

    // Override moduleCommand to handle incoming commands
    virtual int moduleCommand(char* pbuf, size_t len) override {
        return 0;
    }

    // Main Process
    virtual void mainProc() override;

    // Opens network adapters
    virtual int configureAdapter(NetworkAdapter& netAdapter, int adapterIdx) override;

    // Override startReceive_ to route incoming data via UDP
    virtual void configureReceiveCallback(NetworkAdapter& adapter, std::function<void(const uint8_t* data, size_t& length)> dataReceivedCommand_, bool asyncTx=true) override;

    // Override transmitData_ to route data via UDP
    virtual int transmitData_(const uint8_t* data, size_t length) override;

    // Provide host IP lookup for bound adapters
    virtual std::string getHostIP_(NetworkAdapter& adapter) override;

    // Map caller module name -> configured port for fast routing
    unordered_map<string, int> m_CallerPortMap;

    // Non-owning pointer to the primary socket (first configured adapter)
    Network::UdpServer* m_UdpSocket{nullptr};
};
};