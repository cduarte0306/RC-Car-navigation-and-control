#include "NetworkLib.hpp"


namespace NetUtils {

const std::string EthAdpt("enP8p1s0");
const std::string WlanAdpt("wlP1p1s0");

NetworkPort<Network::UdpServer>::NetworkPort(boost::asio::io_context& ioContext, int srcPort, int dstPort, size_t bufferSize_) : 
PortManager(ioContext, srcPort, dstPort) {
    bufferSize = bufferSize_;
    m_Eth = std::make_unique<Network::UdpServer>(ioContext, "enP8p1s0", srcPort,  dstPort);
    m_Wlan = std::make_unique<Network::UdpServer>(ioContext, "wlP1p1s0", srcPort, dstPort);
}

NetworkPort<Network::UdpServer>::NetworkPort(boost::asio::io_context& ioContext) : 
PortManager(ioContext, 0, 0) {
}

NetworkPort<Network::UdpServer>::~NetworkPort() {
}

int NetworkPort<Network::UdpServer>::open(int srcPort, int dstPort) {
    if (!m_Eth || !m_Wlan) {
        return -1;
    }

    std::string ethAdapter = EthAdpt;
    std::string wlanAdapter = WlanAdpt;

    int ret = false;
    const bool ethOk = m_Eth->openSocket(ethAdapter, srcPort, dstPort, bufferSize, false);
    const bool wlanOk = m_Wlan->openSocket(wlanAdapter, srcPort, dstPort, bufferSize, false);
    return (ethOk && wlanOk) ? 0 : -1;
}

Network::UdpServer* NetworkPort<Network::UdpServer>::eth() const {
    return m_Eth.get();
}

Network::UdpServer* NetworkPort<Network::UdpServer>::wlan() const {
    return m_Wlan.get();
}

Network::UdpServer* NetworkPort<Network::UdpServer>::preferred() const {
    return m_Eth ? m_Eth.get() : m_Wlan.get();
}

bool NetworkPort<Network::UdpServer>::hasEth() const {
    return static_cast<bool>(m_Eth);
}

bool NetworkPort<Network::UdpServer>::hasWlan() const {
    return static_cast<bool>(m_Wlan);
}

int NetworkPort<Network::UdpServer>::sendEth(std::string& targetIP, const std::vector<char>& data) {
    if (targetIP.empty()) return -1;
    
    if (m_Eth) {
        m_Eth->transmit(reinterpret_cast<const uint8_t*>(data.data()), data.size(), targetIP);
    } else {
        return -1; // Ethernet interface not available
    }
    return 0;
}

int NetworkPort<Network::UdpServer>::sendWlan(std::string& targetIP, const std::vector<char>& data) {
    if (targetIP.empty()) return -1;
    if (m_Wlan) {
        m_Wlan->transmit(reinterpret_cast<const uint8_t*>(data.data()), data.size(), targetIP);
    } else {
        return -1; // WiFi interface not available
    }
    return 0;
}

int NetworkPort<Network::UdpServer>::setReceiveCallback(std::function<void(std::vector<char>&)> callback) {
    if (m_Eth) {
        m_Eth->startReceive(callback);
    }
    if (m_Wlan) {
        m_Wlan->startReceive(callback);
    }
    return 0;
}

std::string NetworkPort<Network::UdpServer>::getHostIP() {
    if (m_Eth) {
        return m_Eth->getHostIP();
    }
    if (m_Wlan) {
        return m_Wlan->getHostIP();
    }
    return "";
}

NetworkPort<Network::TcpServer>::NetworkPort(boost::asio::io_context& ioContext, int srcPort, int dstPort, size_t bufferSize_) : 
PortManager(ioContext, srcPort, dstPort) {
    bufferSize = bufferSize_;
     m_Eth = std::make_unique<Network::TcpServer>(ioContext, "enP8p1s0", "wlP1p1s0", srcPort, dstPort);
     m_Wlan = std::make_unique<Network::TcpServer>(ioContext, "wlP1p1s0", "enP8p1s0", srcPort, dstPort);
}

NetworkPort<Network::TcpServer>::NetworkPort(boost::asio::io_context& ioContext) : 
PortManager(ioContext, 0, 0) {
}

NetworkPort<Network::TcpServer>::~NetworkPort() {
}

int NetworkPort<Network::TcpServer>::open(int srcPort, int dstPort) {
    if (!m_Eth || !m_Wlan) {
        return -1;
    }

    std::string ethAdapter = EthAdpt;
    std::string wlanAdapter = WlanAdpt;

    const bool ethOk = m_Eth->openSocket(ethAdapter, srcPort, dstPort, bufferSize, false);
    const bool wlanOk = m_Wlan->openSocket(wlanAdapter, srcPort, dstPort, bufferSize, false);
    return (ethOk && wlanOk) ? 0 : -1;
}

Network::TcpServer* NetworkPort<Network::TcpServer>::eth() const {
    return m_Eth.get();
}

Network::TcpServer* NetworkPort<Network::TcpServer>::wlan() const {
    return m_Wlan.get();
}

Network::TcpServer* NetworkPort<Network::TcpServer>::preferred() const {
    return m_Eth ? m_Eth.get() : m_Wlan.get();
}

bool NetworkPort<Network::TcpServer>::hasEth() const {
    return static_cast<bool>(m_Eth);
}

bool NetworkPort<Network::TcpServer>::hasWlan() const {
    return static_cast<bool>(m_Wlan);
}

int NetworkPort<Network::TcpServer>::sendEth(std::string& targetIP, const std::vector<char>& data) {
    if (m_Eth) {
        m_Eth->transmit(reinterpret_cast<const uint8_t*>(data.data()), data.size());
    } else {
        return -1; // Ethernet interface not available
    }
    return 0;
}

int NetworkPort<Network::TcpServer>::sendWlan(std::string& targetIP, const std::vector<char>& data) {
    if (m_Wlan) {
        m_Wlan->transmit(reinterpret_cast<const uint8_t*>(data.data()), data.size());
    } else {
        return -1; // WiFi interface not available
    }
    return 0;
}

int NetworkPort<Network::TcpServer>::setReceiveCallback(std::function<void(std::vector<char>&)> callback) {
    if (m_Eth) {
        m_Eth->startReceive(callback);
    }
    if (m_Wlan) {
        m_Wlan->startReceive(callback);
    }
    return 0;
}

std::string NetworkPort<Network::TcpServer>::getHostIP() {
    if (m_Eth) {
        return m_Eth->getHostIP();
    }
    if (m_Wlan) {
        return m_Wlan->getHostIP();
    }
    return "";
}
} // namespace NetUtils