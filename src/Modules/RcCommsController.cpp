#include "RcCommsController.hpp"
#include <functional>
#include <sstream>
#include <fstream>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <sys/socket.h>
#include "utils/logger.hpp"


using namespace std;
using namespace Adapter;

namespace Modules {

std::vector<std::string> hostMap = {"", ""};

NetworkComms::NetworkComms(int moduleID, string name) 
    : Base(moduleID, name), Adapter::CommsAdapter(name) {
    // Socket instances are created per adapter in configureAdapter.
    setPeriod(1000);  // Set the timer thread to service ever second
}


NetworkComms::~NetworkComms() {
    // m_UdpSockets own sockets; m_UdpSocket is non-owning.
    m_UdpSocket = nullptr;
}


int NetworkComms::init(void) {
    Logger* logger = Logger::getLoggerInst();

    // Initialize host IP map
    hostMap[WlanAdapter] = "";
    hostMap[EthAdapter]  = "";

    m_WlanSocket = std::make_shared<Network::UdpServer>(
        io_context, "wlP1p1s0", "enP8p1s0", 0, WlanHandshakePort);  // Default bakeup interface is enP8p1s0

    m_EthSocket = std::make_shared<Network::UdpServer>(
        io_context, "enP8p1s0", "enP8p1s0", 0, EthHandshakePort);  // Default bakeup interface is wlan

    m_WlanSocket->startReceive(std::bind(&NetworkComms::OnWlanHandShakeRecv, this, std::placeholders::_1));
    m_EthSocket->startReceive(std::bind(&NetworkComms::OnEthHandShakeRecv, this, std::placeholders::_1));

    logger->log(Logger::LOG_LVL_INFO, "NetworkComms module initialized\r\n");
    return 0;
}


/**
 * @brief Handler for handshake data received via UDP
 * 
 * @param data Reference to received data buffer
 */
void NetworkComms::OnWlanHandShakeRecv(std::vector<char>& data) {
    Logger* logger = Logger::getLoggerInst();
    // Process handshake data
    logger->log(Logger::LOG_LVL_INFO, "Received handshake data (%zu bytes)\r\n", data.size());

    // Build reply
    data.clear();
    std::string replyStr = "HANDSHAKE_ACK";

    // Fill the host map for WLAN
    hostMap[NetworkComms::WlanAdapter] = m_WlanSocket->getHostIP();

    if (!hostMap[NetworkComms::WlanAdapter].length())
        logger->log(Logger::LOG_LVL_INFO, "WLAN Host IP: %s\r\n", hostMap[NetworkComms::WlanAdapter].c_str());
    data.insert(data.end(), replyStr.begin(), replyStr.end());
}


/**
 * @brief Handler for handshake data received via UDP
 * 
 * @param data Reference to received data buffer
 */
void NetworkComms::OnEthHandShakeRecv(std::vector<char>& data) {
    Logger* logger = Logger::getLoggerInst();
    // Process handshake data
    logger->log(Logger::LOG_LVL_INFO, "Received handshake data (%zu bytes)\r\n", data.size());

    // Build reply
    data.clear();
    std::string replyStr = "HANDSHAKE_ACK";
    // Fill the host map for Ethernet
    hostMap[NetworkComms::EthAdapter] = m_EthSocket->getHostIP();
    if (!hostMap[NetworkComms::EthAdapter].length())
        logger->log(Logger::LOG_LVL_INFO, "Ethernet Host IP: %s\r\n", hostMap[NetworkComms::EthAdapter].c_str());
    data.insert(data.end(), replyStr.begin(), replyStr.end());
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
    std::unique_ptr<Network::UdpServer> udpSocket;
    try {
        udpSocket = make_unique<Network::UdpServer>(
            io_context, adapter, "enP8p1s0", netAdapter.sPort, netAdapter.dPort, netAdapter.bufferSize, netAdapter.broadcast);  // Default bakeup interface is enP8p1s0
    } catch(const std::exception& e) {
        Logger* logger = Logger::getLoggerInst();
        logger->log(Logger::LOG_LVL_ERROR, "Failed to create UDP socket for adapter %s: %s\r\n", adapter.c_str(), e.what());
        std::pair<int, Adapter::CommsAdapter::NetworkAdapter*> adapterInfo{adapterIdx, &netAdapter};
        m_FailedAdapters.push_back(adapterInfo);
        m_FailedAdapterMap[adapterIdx] = &netAdapter;
        return -1;
    }

    // Set up the transmit callback
    netAdapter.sendCallback = [this, udpSocket=udpSocket.get(), &netAdapter](std::string destIp, const uint8_t* data, size_t length) {
        if (!data || length == 0 || !netAdapter.connected) return -1;

        if (!udpSocket) return -1;

        // UdpServer::transmit expects a non-const buffer pointer
        uint8_t* buf = const_cast<uint8_t*>(data);
        std::string destIp_;
        
        if (destIp.length())
            destIp_ = destIp;
        else
            destIp_ = hostMap[netAdapter.adapterType];
        
        bool ok = udpSocket->transmit(buf, length, destIp_);
        return ok ? 0 : -1;
    };

    netAdapter.hostResolver = [udpSocket=udpSocket.get()]() -> std::string {
        if (!udpSocket) {
            return std::string();
        }
        return udpSocket->getHostIP();
    };

    if (netAdapter.adapter == "enP8p1s0") {
        netAdapter.adapterType = NetworkComms::EthAdapter;
    } else if (netAdapter.adapter == "wlP1p1s0") {
        netAdapter.adapterType = NetworkComms::WlanAdapter;
    }

    netAdapter.sPort = udpSocket->getSrcPort();;
    netAdapter.dPort = udpSocket->getDstPort();

    // Keep ownership in the map and cache a non-owning pointer to the first socket
    Network::UdpServer* socketPtr = udpSocket.get();
    m_UdpSockets[adapterIdx].socket = std::move(udpSocket);
    m_UdpSockets[adapterIdx].sPort   = netAdapter.sPort;
    m_UdpSockets[adapterIdx].dPort   = netAdapter.dPort;
    m_UdpSockets[adapterIdx].moduleName = netAdapter.parent;
    m_UdpSockets[adapterIdx].netAdapter = &netAdapter;

    if (!m_UdpSocket) {
        m_UdpSocket = socketPtr;
    }

    // Map adapter name to the same non-owning socket pointer, without taking ownership again
    if (m_AdapterMap.find(adapter) == m_AdapterMap.end()) {
        m_AdapterMap[adapter] = socketPtr;
    }

    netAdapter.connected = true;
    return 0;
}


/**
 * @brief Configure the receive callback for the network adapter
 * 
 * @param adapter Reference to network adapter struct
 * @param dataReceivedCommand_ Function to process received data
 */
void NetworkComms::configureReceiveCallback(NetworkAdapter& adapter, std::function<void(std::vector<char>&)> dataReceivedCommand_, bool asyncTx) {
    auto it = m_UdpSockets.find(adapter.id);
    if (it == m_UdpSockets.end() || !it->second.socket) {
        Logger* logger = Logger::getLoggerInst();
        logger->log(Logger::LOG_LVL_ERROR, "No UDP socket for adapter %d\r\n", adapter.id);
        return;
    }

    // Start listening for inbound payloads using the provided handler
    it->second.socket->startReceive(std::move(dataReceivedCommand_), asyncTx);
}


/**
 * @brief Write data via WLAN adapter
 * 
 * @param data Pointer to data buffer
 * @param length Size of data
 * @return int Status code
 */
int NetworkComms::wlanWrite(const uint8_t* data, size_t length) {
    if (!data || length == 0) return -1;

    if (m_WlanHostIP.empty()) return -1;

    // UdpServer::transmit expects a non-const buffer pointer
    uint8_t* buf = const_cast<uint8_t*>(data);
    bool ok = this->m_UdpSocket->transmit(buf, length, m_WlanHostIP);
    return ok ? 0 : -1;
}


/**
 * @brief Write data via Ethernet adapter
 * 
 * @param data Pointer to data buffer
 * @param length Size of data
 * @return int Status code
 */
int NetworkComms::ethWrite(const uint8_t* data, size_t length) {
    if (!data || length == 0) return -1;

    if (m_EthHostIP.empty()) return -1;

    // UdpServer::transmit expects a non-const buffer pointer
    uint8_t* buf = const_cast<uint8_t*>(data);
    bool ok = this->m_UdpSocket->transmit(buf, length, m_EthHostIP);
    return ok ? 0 : -1;
}


/**
 * @brief Returns network stats for CLI
 * 
 * @return std::string 
 */
std::string NetworkComms::readStats() {
    std::stringstream stats;

    for (auto& [index, socket] : m_UdpSockets) {
        std::string txPrint;
        std::string rxPrint;

        if (socket.txRate < 1.0) {
            txPrint = std::to_string(socket.txRate * 1000.0) + " Kbps";
        } else {
            txPrint = std::to_string(socket.txRate) + " Mbps";
        }

        if (socket.rxRate < 1.0) {
            rxPrint = std::to_string(socket.rxRate * 1000.0) + " Kbps";
        } else {
            rxPrint = std::to_string(socket.rxRate) + " Mbps";
        } 

        stats << "Module: " << socket.moduleName << " | "
              << "Adapter: " << socket.netAdapter->adapter << " | "
              << "sPort: " << socket.sPort << " | "
              << "dPort: " << socket.dPort << " | "
              << "TX Rate: " << txPrint << " | "
              << "RX Rate: " << rxPrint << "\r\n";
    }

    std::string ret = stats.str();
    return ret;
}


/**
 * @brief Timer timeout callback
 * 
 */
void NetworkComms::OnTimer(void) {
    for (auto& [idx, socket] : m_UdpSockets) {
        socket.txRate = (static_cast<double>(socket.socket->GetTxBytes()) * 8.0) / 1000000.0;
        socket.rxRate = (static_cast<double>(socket.socket->GetRxBytes()) * 8.0) / 1000000.0;

        socket.socket->resetCounters();
    }
}


/**
 * @brief Main processing loop for network communications
 * 
 */
void NetworkComms::mainProc() {
    string hostIP("");
    string lastHost("");

    auto ethernetCablePlugged = [](const char* ifname)
    {
        std::ifstream f(std::string("/sys/class/net/") + ifname + "/carrier");
        int carrier = 0;
        return (f >> carrier) && (carrier == 1);
    };

    auto seEthState = [this](bool state) {
        for (auto& [idx, adapter] : m_UdpSockets) {
            adapter.netAdapter->OnEthLinkDetected(state);
        }
    };

    int ethernetPollCounter = 0;
    int hostIPPollCounter = 0;

    bool lastReading = false;

    while (true) {
        for (auto& [adapterName, udpSocketPtr] : m_AdapterMap) {
            if (!udpSocketPtr) {
                continue;
            }

            if (adapterName.find("wl") != string::npos) {
                // WLAN adapter
                m_WlanHostIP = udpSocketPtr->getHostIP();
            } else if (adapterName.find("en") != string::npos) {
                // Ethernet adapter
                m_EthHostIP = udpSocketPtr->getHostIP();
            }
        }

        for (const auto& [adapterIdx, netAdapterPtr] : m_FailedAdapterMap) {
            std::string& adapter = netAdapterPtr->adapter;
            std::optional<std::string> res = Sockets::findInterface(adapter.c_str());
            if (res.has_value()) {
                int ret = configureAdapter(*netAdapterPtr, adapterIdx);
                if (ret == 0) {
                    // Successfully reconfigured; remove from failed list
                    Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "Successfully reconfigured adapter %s\r\n", netAdapterPtr->adapter.c_str());
                    m_FailedAdapterMap.erase(adapterIdx);
                    break;
                }
            }
        }

        // Check for ethernet link
        if (ethernetPollCounter ++ > 10) {
            bool ret = ethernetCablePlugged("enP8p1s0");
            if (ret != lastReading && ret) {
                Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "Ethernet link detected\r\n");
                seEthState(true);
                lastReading = ret;
            } else if (!ret && ret != lastReading) {
                Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "Ethernet link disconnection detected\r\n");
                seEthState(false);
                lastReading = ret;
            }

            ethernetPollCounter = 0;
        }

        for ( auto& [idx, adapter] : m_UdpSockets ) {
            if (adapter.netAdapter->adapter == "enP8p1s0") {
                if (adapter.socket->getHostIP().length() > 0  && hostMap[NetworkComms::WlanAdapter].length() == 0) {
                    hostMap[NetworkComms::EthAdapter] = adapter.socket->getHostIP();
                    Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "Ethernet Host IP: %s\r\n", hostMap[NetworkComms::EthAdapter].c_str());
                }
            } else if (adapter.netAdapter->adapter == "wlP1p1s0") {
                if (adapter.socket->getHostIP().length() > 0 && hostMap[NetworkComms::WlanAdapter].length() == 0) {
                    hostMap[NetworkComms::WlanAdapter] = adapter.socket->getHostIP();
                    Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "WLAN Host IP: %s\r\n", hostMap[NetworkComms::WlanAdapter].c_str());
                }
            }
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


/**
 * @brief Returns the host IP address
 * 
 * @param adapter Reference to module adapter
 * @return std::string 
 */
std::string NetworkComms::getHostIP_(NetworkAdapter& adapter) {
    auto it = m_UdpSockets.find(adapter.id);
    if (it == m_UdpSockets.end() || !it->second.socket) {
        return std::string();
    }
    return it->second.socket->getHostIP();
}

} // namespace Modules