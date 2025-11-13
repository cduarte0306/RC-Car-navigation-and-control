#include <thread>
#include <mutex>
#include <atomic>
#include <types.h>
#include "RcBase.hpp"

#include "Devices/network_interface/UdpServer.hpp"


namespace Modules {
class WirelessComms : public Base, public Adapter::CommsAdapter {
public:
    WirelessComms(int moduleID, std::string name);
    ~WirelessComms();

    virtual int stop(void) override {
        // Implementation to stop the motor controller
        return 0;
    }

    Adapter::AdapterBase* getInputAdapter() override {
        // WirelessComms may not implement a Motor in-adapter itself. If
        // it has an owned baseAdapter, return that; otherwise return nullptr.
        if (baseAdapter) return baseAdapter.get();
        return nullptr;
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

    virtual int moduleCommand(char* pbuf, size_t len) override {
        return 0;
    }

    virtual void mainProc() override {
        // Main processing loop for the motor controller
        while (true) {
            // Process motor commands
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void processIncomingData(const uint8_t* data, size_t length);

    std::unique_ptr<Network::UdpServer> m_UdpSocket;
};
};