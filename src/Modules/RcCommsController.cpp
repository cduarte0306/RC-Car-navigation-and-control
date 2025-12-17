#include "RcCommsController.hpp"
#include <functional>
#include "utils/logger.hpp"

using namespace std;
using namespace Adapter;

namespace Modules {

NetworkComms::NetworkComms(int moduleID, string name) 
    : Base(moduleID, name), Adapter::CommsAdapter(name) {
    // Socket instances are created per adapter in configureAdapter.
}


NetworkComms::~NetworkComms() {
    // m_UdpSockets own sockets; m_UdpSocket is non-owning.
    m_UdpSocket = nullptr;
}


/**
 * @brief Configure network adapter with port mapping
 * 
 * @param netAdapter Reference to network adapter struct
 * @param adapterIdx Adapter index
 * @return unique_ptr<NetworkAdapter> Configured network adapter
 */
int NetworkComms::configureAdapter(
    Adapter::CommsAdapter::NetworkAdapter& netAdapter, int adapterIdx) {
    
    std::string& adapter = netAdapter.adapter;

    // Dynamically create the UDP object and connect the members from the NeworkComms
    std::unique_ptr<Network::UdpServer> udpSocket = make_unique<Network::UdpServer>(
        io_context, adapter, "enP8p1s0", netAdapter.port, netAdapter.bufferSize);  // Default bakeup interface is enP8p1s0

    // Set up the transmit callback
    netAdapter.sendCallback = [this, udpSocket=udpSocket.get()](std::string destIp, const uint8_t* data, size_t length) {
        if (!data || length == 0) return -1;

        if (!udpSocket) return -1;

        // UdpServer::transmit expects a non-const buffer pointer
        uint8_t* buf = const_cast<uint8_t*>(data);
        bool ok = udpSocket->transmit(buf, length, destIp);
        return ok ? 0 : -1;
    };

    netAdapter.hostResolver = [udpSocket=udpSocket.get()]() -> std::string {
        if (!udpSocket) {
            return std::string();
        }
        return udpSocket->getHostIP();
    };

    // Keep ownership in the map and cache a non-owning pointer to the first socket
    Network::UdpServer* socketPtr = udpSocket.get();
    m_UdpSockets[adapterIdx] = move(udpSocket);
    if (!m_UdpSocket) {
        m_UdpSocket = socketPtr;
    }
    return 0;
}


/**
 * @brief Configure the receive callback for the network adapter
 * 
 * @param adapter Reference to network adapter struct
 * @param dataReceivedCommand_ Function to process received data
 */
void NetworkComms::configureReceiveCallback(NetworkAdapter& adapter, std::function<void(const uint8_t* data, size_t& length)> dataReceivedCommand_, bool asyncTx) {
    auto it = m_UdpSockets.find(adapter.id);
    if (it == m_UdpSockets.end() || !it->second) {
        Logger* logger = Logger::getLoggerInst();
        logger->log(Logger::LOG_LVL_ERROR, "No UDP socket for adapter %d\r\n", adapter.id);
        return;
    }

    // Start listening for inbound payloads using the provided handler
    it->second->startReceive(std::move(dataReceivedCommand_), asyncTx);
}


/**
 * @brief Main processing loop for network communications
 * 
 */
void NetworkComms::mainProc() {
    string hostIP("");
    string lastHost("");

    while (true) {
        // Process motor commands
        hostIP = this->m_UdpSocket->getHostIP();
        if (hostIP.length() && (hostIP != lastHost)) {
            if (this->CameraAdapter) {
                this->CameraAdapter->configurePipeline(hostIP);
            }
            lastHost = hostIP;
        }

        this_thread::sleep_for(chrono::milliseconds(100));
    }
}


/**
 * @brief Override transmitData_ to route via UDP socket using cached port/host
 * 
 * @param data Pointer to data buffer
 * @param length Size of data
 * @return int Status code
 */
int NetworkComms::transmitData_(const uint8_t* data, size_t length) {
    if (!data || length == 0) return -1;

    if (!this->m_UdpSocket) return -1;

    string host = this->m_UdpSocket->getHostIP();
    if (host.empty()) return -1;

    // UdpServer::transmit expects a non-const buffer pointer
    uint8_t* buf = const_cast<uint8_t*>(data);
    bool ok = this->m_UdpSocket->transmit(buf, length, host);
    return ok ? 0 : -1;
}

std::string NetworkComms::getHostIP_(NetworkAdapter& adapter) {
    auto it = m_UdpSockets.find(adapter.id);
    if (it == m_UdpSockets.end() || !it->second) {
        return std::string();
    }
    return it->second->getHostIP();
}

} // namespace Modules