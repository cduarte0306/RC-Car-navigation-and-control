#include <thread>
#include <mutex>
#include <atomic>
#include <types.h>
#include "RcBase.hpp"

#include "Devices/network_interface/UdpServer.hpp"


using namespace Adapter;

namespace Modules {
class CommandController : public Base, public Adapter::CommandAdapter {
public:
    CommandController(int moduleID, std::string name);
    ~CommandController();

    virtual int stop(void) override {
        // Implementation to stop the motor controller
        return 0;
    }

    virtual int init(void) override;

    Adapter::AdapterBase* getInputAdapter() override {
        // CommandController may not implement a Motor in-adapter itself. If
        // it has an owned baseAdapter, return that; otherwise return nullptr.
        if (baseAdapter) return baseAdapter.get();
        return nullptr;
    }
protected:
    enum {
        CmdNoop,         // No operation command
        CmdFwdDir,       // Forward direction command
        CmdSteer,        // Steering command
        CmdCameraSetMode // Camera mode setting command
    };

    typedef struct __attribute__((__packed__))
    {
        uint32_t   sequence_id;
        uint16_t   msg_length;

        struct __attribute__((__packed__))
        {
            uint8_t    command;
            val_type_t data;
            uint32_t   payloadLen;
        } payload;
    } ClientReq_t;

    typedef struct __attribute__((__packed__)) {
        val_type_t data;
        uint8_t state;  
    } reply_t;

    virtual int moduleCommand(char* pbuf, size_t len) override {
        return 0;
    }

    // Main Process
    virtual void mainProc() override;

    // Processes reply from client
    void processIncomingData(const uint8_t* data, size_t& length);

    // Map of UDP sockets by adapter ID
    std::unique_ptr<Adapter::CommsAdapter::NetworkAdapter> m_CommandNetAdapter{nullptr};

    std::unique_ptr<Network::UdpServer> m_UdpSocket;

    // UDP socket
    std::unique_ptr<CommsAdapter::NetworkAdapter> m_CommandAdapter{nullptr};    
};
};