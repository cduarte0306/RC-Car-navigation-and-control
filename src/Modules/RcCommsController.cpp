#include "RcCommsController.hpp"
#include <functional>
#include <sstream>
#include <fstream>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <sys/socket.h>
#include <nlohmann/json.hpp>
#include "utils/logger.hpp"

using namespace std;
using namespace Adapter;

namespace Modules {


static constexpr uint8_t HostConnectionTimeout = 3; // 3 seconds of no contact 
static const std::string replyStr = "HANDSHAKE_ACK";


static std::atomic<bool> hostDetected{false};
static std::atomic<bool> wlanLinkDetected{false};
static std::atomic<bool> ethLinkDetected{false};

static uint8_t hostTmr = 0;


NetworkComms::NetworkComms(ModuleDefs::DeviceType moduleID, std::string name) 
    : Base(moduleID, name), Adapter::CommsAdapter(name) {
    setInputAdapter(static_cast<Adapter::AdapterBase*>(static_cast<Adapter::CommsAdapter*>(this)));

    // Socket instances are created per adapter in configureUDPAdapter.
    setPeriod(1000);  // Set the timer thread to service ever second

    // Instantiate cmd pool
    m_LastCommandSource = new uint8_t[UINT16_MAX];
    for (size_t index = 0; index < UINT16_MAX; ++index) {
        m_LastCommandSource[index] = MaxAdapter;
    }
}


NetworkComms::~NetworkComms() {
    // m_OpenedSockets own sockets; m_UdpSocket is non-owning.
    delete[] m_LastCommandSource;
    m_UdpSocket = nullptr;
}


int NetworkComms::init(void) {
    Logger* logger = Logger::getLoggerInst();
    constexpr unsigned short kHandshakePort = static_cast<unsigned short>(ModuleDefs::NetworkPorts::HandshakePort);
    constexpr unsigned short kCommandDispatcherPort = static_cast<unsigned short>(ModuleDefs::NetworkPorts::CommandDispatcherPort);

    // Initialize host IP map
    hostMap[WlanAdapter] = "";
    hostMap[EthAdapter]  = "";

    m_WlanSocket = std::make_shared<Network::UdpServer>(
        io_context, "wlP1p1s0", kHandshakePort, 0);  // Use ephemeral local port to avoid colliding with ETH handshake listener

    m_EthSocket = std::make_shared<Network::UdpServer>(
        io_context, "enP8p1s0", kHandshakePort, 0);  // Broadcast receive on handshake port

    // Module command dispatchers
    m_EthCmdDispatcher = std::make_shared<Network::UdpServer>(
        io_context, "enP8p1s0", kCommandDispatcherPort, 0);  // Default bakeup interface is wlan

    m_WlanCmdDispatcher = std::make_shared<Network::UdpServer>(
        io_context, "wlP1p1s0", kCommandDispatcherPort, 0);  // Default bakeup interface is enP8p1s0

    m_WlanSocket->startReceive(std::bind(&NetworkComms::OnWlanHandShakeRecv, this, std::placeholders::_1));
    m_EthSocket->startReceive( std::bind(&NetworkComms::OnEthHandShakeRecv,  this, std::placeholders::_1));

    // Initialize command dispatcher ports. Disabling asynch reply to ensure replies are only handed by modules
    m_EthCmdDispatcher->startReceive( std::bind(&NetworkComms::OnEthCmdRecv,  this, std::placeholders::_1));
    m_WlanCmdDispatcher->startReceive(std::bind(&NetworkComms::OnWlanCmdRecv, this, std::placeholders::_1));

    logger->log(Logger::LOG_LVL_INFO, "NetworkComms module initialized\r\n");
    return 0;
}


void NetworkComms::OnEthCmdRecv(std::vector<char>& data) {
    Logger* logger = Logger::getLoggerInst();
    // Submit to pool
    BaseMsgHdr* hdr = reinterpret_cast<BaseMsgHdr*>(data.data());
    if (hdr->seqID >= UINT16_MAX) {
        logger->log(Logger::LOG_LVL_ERROR, "Received command with invalid sequence ID: %d\r\n", hdr->seqID);
        return;
    }

    hostTmr = HostConnectionTimeout;
    
    m_LastCommandSource[hdr->seqID] = EthAdapter;
    if (m_EthCmdDispatcher) {
        m_EthCmdDispatcher->setRemoteEndpoint(m_EthCmdDispatcher->getHostIP(), m_EthCmdDispatcher->getPort());
    }
    // Process command data received on Ethernet
    Base::dispatchToModule(data);
}


void NetworkComms::OnWlanCmdRecv(std::vector<char>& data) {
    Logger* logger = Logger::getLoggerInst();
    // Process command data received on WLAN
    BaseMsgHdr* hdr = reinterpret_cast<BaseMsgHdr*>(data.data());
    if (hdr->seqID >= UINT16_MAX) {
        logger->log(Logger::LOG_LVL_ERROR, "Received command with invalid sequence ID: %d\r\n", hdr->seqID);
        return;
    }

    hostTmr = HostConnectionTimeout;

    // Submit to pool
    m_LastCommandSource[hdr->seqID] = WlanAdapter;
    if (m_WlanCmdDispatcher) {
        m_WlanCmdDispatcher->setRemoteEndpoint(m_WlanCmdDispatcher->getHostIP(), m_WlanCmdDispatcher->getPort());
    }
    Base::dispatchToModule(data);
}


int NetworkComms::OnReply(Msg::MessageAck<std::vector<char>>& ack) {
    // This function is called by the reply processing thread to handle any messages that are sent back from the module to the adapter as part of command acknowledgments or responses. The adapter can implement this function to process the reply messages and take appropriate actions based on the content of the replies.
    Logger* logger = Logger::getLoggerInst();
    if (ack.mSeqID >= UINT16_MAX) {
        logger->log(Logger::LOG_LVL_ERROR, "Invalid sequence ID for reply: %d\r\n", ack.mSeqID);
        return -1;
    }

    uint8_t adapterId = !hostMap[EthAdapter].empty() ? EthAdapter : WlanAdapter;
    std::shared_ptr<Network::UdpServer> socket = (adapterId == EthAdapter) ? m_EthCmdDispatcher : m_WlanCmdDispatcher;
    const char* adapterName = (adapterId == EthAdapter) ? "[ETH]" : "[WLAN]";
    if (!socket) {
        logger->log(Logger::LOG_LVL_ERROR, "No reply socket available for adapter %s\r\n", adapterName);
        return -1;
    }

    std::string destIP = hostMap[adapterId];
    if (destIP.empty()) {
        destIP = socket->getHostIP();
    }

    if (destIP.empty() && adapterId == EthAdapter) {
        adapterId = WlanAdapter;
        socket = m_WlanCmdDispatcher;
        adapterName = "[WLAN]";
        if (!socket) {
            logger->log(Logger::LOG_LVL_ERROR, "No reply socket available for adapter %s\r\n", adapterName);
            return -1;
        }
        destIP = hostMap[adapterId];
        if (destIP.empty()) {
            destIP = socket->getHostIP();
        }
    }

    if (destIP.empty()) {
        logger->log(Logger::LOG_LVL_ERROR, "No known host IP for reply adapter %s (seq %d)\r\n", adapterName, ack.mSeqID);
        return -1;
    }

    const std::vector<uint8_t> frame = Msg::CommsPipePacket::serialize(ack);
    logger->log(Logger::LOG_LVL_DEBUG, "%s Processing reply for Command ID %d, Sequence ID %d, Status %s, Dest Host IP: %s:%d\r\n",
                adapterName, ack.mCommandID, ack.mSeqID, ack.GetStatus() ? "Success" : "Failure", destIP.c_str(), socket->getDstPort());

    bool ok = Msg::CommsPipePacket::send(*socket, frame, destIP);
    if (!ok) {
        logger->log(Logger::LOG_LVL_ERROR, "Failed to transmit reply data for sequence ID %d\r\n", ack.mSeqID);
        return -1;
    }
    return 0;
}

int NetworkComms::OnModuleMsgReceived(Msg::MessageCapsule<std::vector<char>>& capsule) {
    // Base handler remains a safe default for module-originated messages.
    return Base::OnModuleMsgReceived(capsule);
}

/**
 * @brief Handler for handshake data received via UDP
 * 
 * @param data Reference to received data buffer
 */
void NetworkComms::OnWlanHandShakeRecv(std::vector<char>& data) {
    Logger* logger = Logger::getLoggerInst();

    // Build reply
    data.clear();
    // Fill the host map for WLAN
    hostMap[NetworkComms::WlanAdapter] = m_WlanSocket->getHostIP();
    if (m_WlanCmdDispatcher) {
        m_WlanCmdDispatcher->setRemoteEndpoint(m_WlanSocket->getHostIP(), m_WlanSocket->getPort());
    }

    if (!hostMap[NetworkComms::WlanAdapter].length())
        logger->log(Logger::LOG_LVL_INFO, "WLAN Host IP: %s\r\n", m_WlanSocket->getHostIP().c_str());
    wlanLinkDetected.store(true);

    // Build reply json
    std::optional<std::string> ethIP = Network::Sockets::findInterface(ETHAdapter);

    nlohmann::json replyJson;
    replyJson["message"] = replyStr;
    replyJson["eth_ip"]  = ethIP.value_or("");
    std::string wlanIfName = WLANAdapter;
    replyJson["net_mask"] = Network::Sockets::getNetMask(wlanIfName);

    const std::string replyPayload = replyJson.dump();
    data.insert(data.end(), replyPayload.begin(), replyPayload.end());
    std::string destIP = m_WlanSocket->getHostIP();
    // Reply over the wlan handshake socket
    logger->log(Logger::LOG_LVL_INFO, "Received handshake data on WLAN (%zu bytes) from %s:%d\r\n", data.size(), destIP.c_str(), m_WlanSocket->getPort());
    bool ok = m_WlanSocket->transmit(reinterpret_cast<const uint8_t*>(data.data()), data.size(), destIP);
}


/**
 * @brief Handler for handshake data received via UDP
 * 
 * @param data Reference to received data buffer
 */
void NetworkComms::OnEthHandShakeRecv(std::vector<char>& data) {
    Logger* logger = Logger::getLoggerInst();

    // Build reply
    data.clear();
    // Fill the host map for Ethernet
    hostMap[NetworkComms::EthAdapter] = m_EthSocket->getHostIP();
    if (m_EthCmdDispatcher) {
        m_EthCmdDispatcher->setRemoteEndpoint(m_EthSocket->getHostIP(), m_EthSocket->getPort());
    }
    if (!hostMap[NetworkComms::EthAdapter].length())
        logger->log(Logger::LOG_LVL_INFO, "Ethernet Host IP: %s\r\n", m_EthSocket->getHostIP().c_str());
    data.insert(data.end(), replyStr.begin(), replyStr.end());
    std::string destIP = m_EthSocket->getHostIP();
    ethLinkDetected.store(true);

    // Reply over the eth handshake socket
    logger->log(Logger::LOG_LVL_INFO, "Received handshake data on Ethernet (%zu bytes) from %s:%d\r\n", data.size(), destIP.c_str(), m_EthSocket->getPort());
    bool ok = m_EthSocket->transmit(reinterpret_cast<const uint8_t*>(data.data()), data.size(), destIP);
}


/**
 * @brief Configure network adapter with port mapping
 * 
 * @param netAdapter Reference to network adapter struct
 * @param adapterIdx Adapter index
 * @return unique_ptr<NetworkAdapter> Configured network adapter
 */
int NetworkComms::configureUDPAdapter(
    Adapter::CommsAdapter::NetworkAdapter& netAdapter, int adapterIdx) {
    std::unique_ptr<NetUtils::NetworkPort<Network::UdpServer>> udpPort;
    Network::UdpServer* selectedSocket = nullptr;

    try {
        udpPort = std::make_unique<NetUtils::NetworkPort<Network::UdpServer>>(
            io_context, netAdapter.sPort, netAdapter.dPort, netAdapter.bufferSize);

        if (netAdapter.adapter.find("wl") == 0) {
            selectedSocket = udpPort->wlan();
        } else if (netAdapter.adapter.find("en") == 0) {
            selectedSocket = udpPort->eth();
        } else {
            selectedSocket = udpPort->preferred();
        }
    } catch(const std::exception& e) {
        Logger* logger = Logger::getLoggerInst();
        logger->log(Logger::LOG_LVL_ERROR, "Failed to create UDP socket for adapter: %s\r\n", e.what());
        std::pair<int, Adapter::CommsAdapter::NetworkAdapter*> adapterInfo{adapterIdx, &netAdapter};
        m_FailedAdapters.push_back(adapterInfo);
        m_FailedAdapterMap[adapterIdx] = &netAdapter;
        return -1;
    }

    if (!selectedSocket) {
        return -1;
    }
    netAdapter.EthPresent = [this](void) -> bool {
        return ethLinkDetected.load();
    };

    netAdapter.hostPresentCB = [this](void) -> bool {
        return hostDetected.load();
    };

    netAdapter.sPort = selectedSocket->getSrcPort();
    netAdapter.dPort = selectedSocket->getDstPort();

    netAdapter.socketDesc = m_RegisteredPorts.size();
    m_RegisteredPorts.insert_or_assign(netAdapter.socketDesc, std::move(udpPort));

    auto registeredPortIt = m_RegisteredPorts.find(netAdapter.socketDesc);
    if (registeredPortIt == m_RegisteredPorts.end() || !registeredPortIt->second) {
        return -1;
    }

    auto& registeredPort = static_cast<NetUtils::NetworkPort<Network::UdpServer>&>(*registeredPortIt->second);

    // Keep a non-owning pointer for runtime access/stats
    Network::Sockets* socketPtr = selectedSocket;
    m_OpenedSockets[adapterIdx].socket  = socketPtr;
    m_OpenedSockets[adapterIdx].sPort   = netAdapter.sPort;
    m_OpenedSockets[adapterIdx].dPort   = netAdapter.dPort;
    m_OpenedSockets[adapterIdx].moduleName = netAdapter.parent;
    m_OpenedSockets[adapterIdx].netAdapter = &netAdapter;

    if (!m_UdpSocket) {
        m_UdpSocket = socketPtr;
    }

    // Map adapter name to the same non-owning socket pointer, without taking ownership again
    if (m_AdapterMap.find(netAdapter.adapter) == m_AdapterMap.end()) {
        m_AdapterMap[netAdapter.adapter] = socketPtr;
    }

    // Set up the transmit callback after the port is owned by m_RegisteredPorts so the reference stays valid.
    netAdapter.sendCallback = [this, &registeredPort, &netAdapter](
        const uint8_t* data, size_t length) -> int {
        if (!data || length == 0 || !netAdapter.connected) return -1;

        uint8_t* buf = const_cast<uint8_t*>(data);
        if (ethLinkDetected.load()) {
            Network::UdpServer* udpSocketEth = registeredPort.eth();
            if (udpSocketEth && !hostMap[NetworkComms::EthAdapter].empty()) {
                std::string& destIp = hostMap[NetworkComms::EthAdapter];
                return udpSocketEth->transmit(buf, length, destIp) ? 0 : -1;
            }
        }

        Network::UdpServer* udpSocketWlan = registeredPort.wlan();
        if (udpSocketWlan && !hostMap[NetworkComms::WlanAdapter].empty()) {
            std::string& destIp = hostMap[NetworkComms::WlanAdapter];
            return udpSocketWlan->transmit(buf, length, destIp) ? 0 : -1;
        }

        return -1;
    };

    netAdapter.sendCallbacEth = [this, &registeredPort, &netAdapter](
        const uint8_t* data, size_t length) {
        if (!data || length == 0 || !netAdapter.connected) return -1;

        Network::UdpServer* udpSocketEth = registeredPort.eth();
        if (!udpSocketEth) return -1;

        uint8_t* buf = const_cast<uint8_t*>(data);
        std::string& destIp_ = hostMap[NetworkComms::EthAdapter];

        if (destIp_.empty()) {
            return -1;
        }

        bool ok = udpSocketEth->transmit(buf, length, destIp_);
        return ok ? 0 : -1;
    };

    netAdapter.sendCallbackWlan = [this, &registeredPort, &netAdapter](
        const uint8_t* data, size_t length) {
        if (!data || length == 0 || !netAdapter.connected) return -1;

        Network::UdpServer* udpSocketWlan = registeredPort.wlan();
        if (!udpSocketWlan) return -1;

        uint8_t* buf = const_cast<uint8_t*>(data);
        std::string& destIp_ = hostMap[NetworkComms::WlanAdapter];

        if (destIp_.empty()) {
            return -1;
        }

        bool ok = udpSocketWlan->transmit(buf, length, destIp_);
        return ok ? 0 : -1;
    };

    netAdapter.preferredSrcPortCb = [this, &registeredPort, &netAdapter]() -> int {
        if (netAdapter.adapter.find("wl") == 0) {
            Network::UdpServer* udpSocketWlan = registeredPort.wlan();
            return udpSocketWlan ? udpSocketWlan->getSrcPort() : netAdapter.sPort;
        }

        if (netAdapter.adapter.find("en") == 0) {
            Network::UdpServer* udpSocketEth = registeredPort.eth();
            return udpSocketEth ? udpSocketEth->getSrcPort() : netAdapter.sPort;
        }

        if (ethLinkDetected.load()) {
            Network::UdpServer* udpSocketEth = registeredPort.eth();
            if (udpSocketEth) {
                return udpSocketEth->getSrcPort();
            }
        }

        Network::UdpServer* udpSocketWlan = registeredPort.wlan();
        if (udpSocketWlan) {
            return udpSocketWlan->getSrcPort();
        }

        return netAdapter.sPort;
    };

    const int socketDesc = netAdapter.socketDesc;
    const std::string adapterName = netAdapter.adapter;
    netAdapter.closeSocketCb = [this, adapterIdx, socketDesc, adapterName]() -> int {
        auto registeredPortIt = m_RegisteredPorts.find(socketDesc);
        if (registeredPortIt != m_RegisteredPorts.end() && registeredPortIt->second) {
            registeredPortIt->second->close();
            m_RegisteredPorts.erase(registeredPortIt);
        }

        m_OpenedSockets.erase(adapterIdx);

        auto adapterMapIt = m_AdapterMap.find(adapterName);
        if (adapterMapIt != m_AdapterMap.end()) {
            m_AdapterMap.erase(adapterMapIt);
        }

        return 0;
    };

    netAdapter.connected = true;

    return 0;
}


/**
 * @brief Configure TCP network adapter
 * 
 * @param netAdapter Reference to network adapter struct
 * @param adapterIdx Adapter index
 */
int NetworkComms::configureTCPAdapter(Adapter::CommsAdapter::NetworkAdapter& netAdapter, int adapterIdx) {
    std::unique_ptr<NetUtils::NetworkPort<Network::TcpServer>> tcpPort;
    Network::TcpServer* selectedSocket = nullptr;

    try {
        tcpPort = std::make_unique<NetUtils::NetworkPort<Network::TcpServer>>(
            io_context, netAdapter.sPort, netAdapter.dPort, netAdapter.bufferSize, netAdapter.adapter == "lo");

        if (netAdapter.adapter == "lo") {
            selectedSocket = tcpPort->hasLo() ? tcpPort->preferred() : nullptr;
        } else if (netAdapter.adapter.find("wl") == 0) {
            selectedSocket = tcpPort->wlan();
        } else if (netAdapter.adapter.find("en") == 0) {
            selectedSocket = tcpPort->eth();
        } else {
            selectedSocket = tcpPort->preferred();
        }
    } catch(const std::exception& e) {
        Logger* logger = Logger::getLoggerInst();
        logger->log(Logger::LOG_LVL_ERROR, "Failed to create TCP socket for adapter %s: %s\r\n", netAdapter.adapter.c_str(), e.what());
        std::pair<int, Adapter::CommsAdapter::NetworkAdapter*> adapterInfo{adapterIdx, &netAdapter};
        m_FailedAdapters.push_back(adapterInfo);
        m_FailedAdapterMap[adapterIdx] = &netAdapter;
        return -1;
    }

    if (!selectedSocket) {
        return -1;
    }
    netAdapter.sPort = selectedSocket->getSrcPort();
    netAdapter.dPort = selectedSocket->getDstPort();

    netAdapter.socketDesc = adapterIdx;
    m_RegisteredPorts.insert_or_assign(netAdapter.socketDesc, std::move(tcpPort));

    auto registeredPortIt = m_RegisteredPorts.find(netAdapter.socketDesc);
    if (registeredPortIt == m_RegisteredPorts.end() || !registeredPortIt->second) {
        return -1;
    }

    auto& registeredPort = static_cast<NetUtils::NetworkPort<Network::TcpServer>&>(*registeredPortIt->second);

    // Keep a non-owning pointer for runtime access/stats
    Network::TcpServer* socketPtr = selectedSocket;
    m_OpenedSockets[adapterIdx].socket  = socketPtr;
    m_OpenedSockets[adapterIdx].sPort   = netAdapter.sPort;
    m_OpenedSockets[adapterIdx].dPort   = netAdapter.dPort;
    m_OpenedSockets[adapterIdx].moduleName = netAdapter.parent;
    m_OpenedSockets[adapterIdx].netAdapter = &netAdapter;

    if (!m_UdpSocket) {
        m_UdpSocket = socketPtr;
    }

    // Map adapter name to the same non-owning socket pointer, without taking ownership again
    if (m_AdapterMap.find(netAdapter.adapter) == m_AdapterMap.end()) {
        m_AdapterMap[netAdapter.adapter] = socketPtr;
    }

    netAdapter.sendCallbackTcp = [this, &registeredPort, &netAdapter](const uint8_t* data, size_t length) {
        if (!data || length == 0 || !netAdapter.connected) return -1;

        uint8_t* buf = const_cast<uint8_t*>(data);

        if (netAdapter.adapter == "lo") {
            Network::TcpServer* tcpSocket = registeredPort.preferred();
            if (!tcpSocket) return -1;
            return tcpSocket->transmit(buf, length) ? 0 : -1;
        }

        // Dual-interface TCP adapters may accept on either ETH or WLAN.
        Network::TcpServer* tcpSocketEth = registeredPort.eth();
        if (tcpSocketEth && tcpSocketEth->transmit(buf, length)) {
            return 0;
        }

        Network::TcpServer* tcpSocketWlan = registeredPort.wlan();
        if (tcpSocketWlan && tcpSocketWlan->transmit(buf, length)) {
            return 0;
        }

        return -1;
    };

    netAdapter.preferredSrcPortCb = [&registeredPort, &netAdapter]() -> int {
        Network::TcpServer* tcpSocket = registeredPort.preferred();
        return tcpSocket ? tcpSocket->getSrcPort() : netAdapter.sPort;
    };

    const int socketDesc = netAdapter.socketDesc;
    const std::string adapterName = netAdapter.adapter;
    netAdapter.closeSocketCb = [this, adapterIdx, socketDesc, adapterName]() -> int {
        auto registeredPortIt = m_RegisteredPorts.find(socketDesc);
        if (registeredPortIt != m_RegisteredPorts.end() && registeredPortIt->second) {
            registeredPortIt->second->close();
            m_RegisteredPorts.erase(registeredPortIt);
        }

        m_OpenedSockets.erase(adapterIdx);

        auto adapterMapIt = m_AdapterMap.find(adapterName);
        if (adapterMapIt != m_AdapterMap.end()) {
            m_AdapterMap.erase(adapterMapIt);
        }

        return 0;
    };

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
    (void)asyncTx; // Currently unused since async transmit was removed, but can be re-enabled in the future if needed.
    auto it = m_OpenedSockets.find(adapter.id);
    if (it == m_OpenedSockets.end() || !it->second.socket) {
        Logger* logger = Logger::getLoggerInst();
        logger->log(Logger::LOG_LVL_ERROR, "No UDP socket for adapter %d\r\n", adapter.id);
        return;
    }

    auto* primaryTcpSocket = dynamic_cast<Network::TcpServer*>(it->second.socket);
    if (primaryTcpSocket) {
        auto registeredPortIt = m_RegisteredPorts.find(adapter.socketDesc);
        if (registeredPortIt != m_RegisteredPorts.end() && registeredPortIt->second) {
            auto& tcpPort = static_cast<NetUtils::NetworkPort<Network::TcpServer>&>(*registeredPortIt->second);

            auto armTcpSocket = [&](Network::TcpServer* tcpSocket) {
                if (!tcpSocket) {
                    return;
                }

                if (adapter.onConnected) {
                    tcpSocket->onConnectionEstablished(adapter.onConnected);
                }

                tcpSocket->startReceive(dataReceivedCommand_);
            };

            if (adapter.adapter == "lo") {
                armTcpSocket(tcpPort.preferred());
            } else {
                armTcpSocket(tcpPort.eth());
                armTcpSocket(tcpPort.wlan());
            }

            return;
        }
    }

    // Start listening for inbound payloads using the provided handler
    it->second.socket->startReceive(std::move(dataReceivedCommand_));
}


void NetworkComms::configureOnConnectCallback(NetworkAdapter& adapter, std::function<void(std::string& hostIp)> callback) {
    auto it = m_OpenedSockets.find(adapter.id);
    if (it == m_OpenedSockets.end() || !it->second.socket) {
        Logger* logger = Logger::getLoggerInst();
        logger->log(Logger::LOG_LVL_ERROR, "No UDP socket for adapter %d\r\n", adapter.id);
        return;
    }

    if (adapter.onConnected) {
        auto* tcp = dynamic_cast<Network::TcpServer*>(it->second.socket);
        if (tcp) {
            tcp->onConnectionEstablished(adapter.onConnected);
        }
    }

    // Start listening for inbound payloads using the provided handler
    // it->second.socket->startReceive(std::move(callback));
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

    for (auto& [index, socket] : m_OpenedSockets) {
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
    static bool hostState = false;
    if (hostTmr > 0) hostTmr --;

    if (hostTmr == 0) hostDetected.store(false);
    else              hostDetected.store(true);

    if (hostDetected && !hostState) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "Host detected\r\n");
        hostState = true;
    } else if (!hostDetected && hostState) {
        Logger::getLoggerInst()->log(Logger::LOG_LVL_INFO, "Host connection timeout\r\n");
        hostState = false;
    }

    for (auto& [idx, socket] : m_OpenedSockets) {
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
        for (auto& [idx, adapter] : m_OpenedSockets) {
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
            std::optional<std::string> res = Network::Sockets::findInterface(adapter.c_str());
            if (res.has_value()) {
                int ret = configureUDPAdapter(*netAdapterPtr, adapterIdx);
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

        for ( auto& [idx, adapter] : m_OpenedSockets ) {
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

} // namespace Modules