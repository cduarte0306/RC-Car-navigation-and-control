#include "RcWirelessComms.hpp"
#include <functional>
#include "utils/logger.hpp"
#include <systemd/sd-bus.h>


#define WLAN_ADAPTER  "wlP1p1s0"
#define LAN_ADAPTER   "enP8p1s0"
#define SERVER_PORT  65000


namespace Modules {
WirelessComms::WirelessComms(int moduleID, std::string name) : Base(moduleID, name) {
    Logger* logger = Logger::getLoggerInst();
    try {
        this->m_UdpSocket = std::make_unique<Network::UdpServer>(io_context, WLAN_ADAPTER, LAN_ADAPTER, SERVER_PORT);
        this->m_UdpSocket->startReceive(std::bind(&WirelessComms::processIncomingData, this, std::placeholders::_1, std::placeholders::_2));
    } catch(std::exception& e) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to open UDP port\r\n");
    }
}


WirelessComms::~WirelessComms() {
    delete this->m_UdpSocket.get();
}


/**
 * @brief Process incoming data received via UDP
 * 
 * @param data Pointer to the data buffer
 * @param length Length of the data buffer
 */
void WirelessComms::processIncomingData(const uint8_t* pData, size_t& length) {
    if (!pData) {
        return;
    }

    Logger* logger = Logger::getLoggerInst();
    ClientReq_t* clientData = reinterpret_cast<ClientReq_t*>(const_cast<uint8_t*>(pData));
    WirelessComms::reply_t reply;
    length = sizeof(reply);

    switch(clientData->payload.command) {
        case CMD_NOOP:
            // No operation command, do nothing
            reply.state = true; // Success
            break;

        case CMD_FWD_DIR:
            reply.data = clientData->payload.data; // Echo back the data
            reply.state = true; // Example state for success
            this->motorAdapter->setMotorSpeed(reply.data.i16); // Example: set speed with 0 direction
            break;

        case CMD_STEER:
            // Handle steering command
            reply.state = true; // Example state for success
            this->motorAdapter->steer(clientData->payload.data.i16); // Example: set steering angle
            break;

        default:
            // Unknown command, set error state
            logger->log(Logger::LOG_LVL_WARN, "Unknown command :%d\r\n", clientData->payload.command);
            reply.state = true; // Error state
            break;
    }
}


/**
 * @brief Main process
 * 
 */
void WirelessComms::mainProc() {
    int r;

    while (true) {
        // Read the adapter name. If the current connected adapter is now Wi-Fi then poll
        // until then adapter becomes available
        std::string& adapterName = this->m_UdpSocket->getAdapterName();
        if (adapterName != WLAN_ADAPTER) {
            // Poll available adapters
            std::optional<std::string> adapter = this->m_UdpSocket->findInterface(WLAN_ADAPTER);
            if (adapter.has_value()) {
                // Re-start ahavi linux
                sd_bus *bus = NULL;
                sd_bus_error error = SD_BUS_ERROR_NULL;
                sd_bus_message *msg = NULL;

                // Connect to the system bus
                r = sd_bus_open_system(&bus);
                if (r < 0) {
                    throw std::runtime_error("Failed to connect to system bus");
                }

                // Call RestartUnit("avahi-daemon.service", "replace")
                r = sd_bus_call_method(
                        bus,
                        "org.freedesktop.systemd1",
                        "/org/freedesktop/systemd1",
                        "org.freedesktop.systemd1.Manager",
                        "RestartUnit",
                        &error,
                        &msg,
                        "ss",
                        "avahi-daemon.service",
                        "replace");

                if (r < 0) {
                    fprintf(stderr, "Failed to restart avahi-daemon: %s\n", error.message);
                } else {
                    printf("avahi-daemon restart requested successfully.\n");
                }

                sd_bus_error_free(&error);
                sd_bus_message_unref(msg);
                sd_bus_unref(bus);

                // Delete the object and restart it
                this->m_UdpSocket.reset();
                this->m_UdpSocket = std::make_unique<Network::UdpServer>(io_context, WLAN_ADAPTER, LAN_ADAPTER, SERVER_PORT);
                this->m_UdpSocket->startReceive(std::bind(&WirelessComms::processIncomingData, this, std::placeholders::_1, std::placeholders::_2));
            }
        }
        // Process motor commands
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
}
};