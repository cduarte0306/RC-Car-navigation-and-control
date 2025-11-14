#include "RcWirelessComms.hpp"
#include <functional>
#include "utils/logger.hpp"


namespace Modules {
WirelessComms::WirelessComms(int moduleID, std::string name) : Base(moduleID, name) {
    this->m_UdpSocket = std::make_unique<Network::UdpServer>(io_context, "wlP1p1s0", "enP8p1s0", 65000);
    this->m_UdpSocket->startReceive(std::bind(&WirelessComms::processIncomingData, this, std::placeholders::_1, std::placeholders::_2));
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
};