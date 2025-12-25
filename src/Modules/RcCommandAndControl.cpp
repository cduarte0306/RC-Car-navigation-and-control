#include "RcCommandAndControl.hpp"

#include <functional>
#include <cstring>

#include "utils/logger.hpp"
#include "Devices/RegisterMap.hpp"


namespace Modules {
CommandController::CommandController(int moduleID, std::string name) : Base(moduleID, name), Adapter::CommandAdapter(name) {
    // this->m_UdpSocket = std::make_unique<Network::UdpServer>(io_context, "wlP1p1s0", "enP8p1s0", 65000);
    // this->m_UdpSocket->startReceive(std::bind(&CommandController::processIncomingData, this, std::placeholders::_1, std::placeholders::_2));
    
}


CommandController::~CommandController() {
    this->m_UdpSocket.reset();
}


/**
 * @brief Initialize the command controller module
 * 
 * @return int Error code
 */
int CommandController::init(void) {
    Logger* logger = Logger::getLoggerInst();
    // Initialize the UDP server
    m_CommandNetAdapter = this->CommsAdapter->createNetworkAdapter(65000, "wlP1p1s0", Adapter::CommsAdapter::MaxUDPPacketSize);
    if (!m_CommandNetAdapter) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to create command network adapter\r\n");
        return -1;
    }

    this->CommsAdapter->startReceive(*m_CommandNetAdapter, std::bind(&CommandController::processIncomingData, this, std::placeholders::_1));
    return 0;
}


/**
 * @brief Process incoming data received via UDP
 * 
 * @param data Pointer to the data buffer
 * @param length Length of the data buffer
 */
void CommandController::processIncomingData(std::vector<char>& buffer) {
    if (buffer.empty()) {
        return;
    }

    Logger* logger = Logger::getLoggerInst();
    if (buffer.size() < sizeof(ClientReq_t)) {
        logger->log(Logger::LOG_LVL_WARN, "Command packet too small (%zu, need %zu)\r\n", buffer.size(), sizeof(ClientReq_t));
        return;
    }

    ClientReq_t* clientData = reinterpret_cast<ClientReq_t*>(buffer.data());

    // Validate declared lengths before accessing payload
    const size_t declaredMsgLen = clientData->msg_length;
    const size_t baseHeader = sizeof(clientData->sequence_id) + sizeof(clientData->msg_length);
    const size_t basePayload = sizeof(clientData->payload);
    const size_t minimumFrame = baseHeader + basePayload;

    if (declaredMsgLen < basePayload) {
        logger->log(Logger::LOG_LVL_WARN, "Command msg_length too small (declared=%zu, base=%zu)\r\n", declaredMsgLen, basePayload);
        return;
    }

    const size_t packetExpected = baseHeader + declaredMsgLen;
    if (buffer.size() < packetExpected) {
        logger->log(Logger::LOG_LVL_WARN, "Command packet truncated (have=%zu, expect=%zu)\r\n", buffer.size(), packetExpected);
        return;
    }

    const size_t extraLen = clientData->payload.payloadLen;
    const size_t extraOffset = sizeof(ClientReq_t);
    if (extraLen > 0 && (extraOffset + extraLen) > buffer.size()) {
        logger->log(Logger::LOG_LVL_WARN, "Command payload length mismatch (field=%zu, remaining=%zu)\r\n", extraLen, buffer.size() - extraOffset);
        return;
    }

    CommandController::reply_t reply;
    size_t length = sizeof(reply);

    switch(clientData->payload.command) {
        case CmdNoop:
            // No operation command, do nothing
            reply.state = true; // Success
            break;

        case CmdFwdDir:
            reply.data = clientData->payload.data; // Echo back the data
            reply.state = true; // Example state for success
            this->motorAdapter->setMotorSpeed(reply.data.i16); // Example: set speed with 0 direction
            break;

        case CmdSteer:
            // Handle steering command
            reply.state = true; // Example state for success
            this->motorAdapter->steer(clientData->payload.data.i16); // Example: set steering angle
            break;

        case CmdCameraSetMode: {
            reply.state = true;
            std::vector<char> payloadBuffer;
            if (extraLen > 0) {
                payloadBuffer.insert(payloadBuffer.end(), buffer.begin() + extraOffset, buffer.begin() + extraOffset + extraLen);
            }
            int ret = this->CameraAdapter->moduleCommand(payloadBuffer);
            reply.state = (ret >= 0);
        } break;

        default:
            // Unknown command, set error state
            logger->log(Logger::LOG_LVL_WARN, "Unknown command :%d\r\n", clientData->payload.command);
            reply.state = true; // Error state
            break;
    }

    // Build reply
    buffer.clear();
    buffer.resize(sizeof(reply));
    std::memcpy(buffer.data(), &reply, sizeof(reply));
}


/**
 * @brief Main processing loop for the motor controller
 * 
 */
void CommandController::mainProc() {
        // Main processing loop for the motor controller
        std::string hostIP("");
        std::string lastHost("");
        RegisterMap* regMap = RegisterMap::getInstance();

        while (true) {
            // Process motor commands
            hostIP = this->CommsAdapter->getHostIP(*m_CommandNetAdapter);
            if (hostIP.length() && (hostIP != lastHost)) {
                this->CameraAdapter->configurePipeline(hostIP);
                regMap->set(RegisterMap::RegisterKeys::HostIP, hostIP);
                lastHost = hostIP;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
};