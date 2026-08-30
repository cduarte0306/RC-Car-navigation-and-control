#include "NetworkLib.hpp"


namespace NetUtils {

const std::string EthAdpt("enP8p1s0");
const std::string WlanAdpt("wlP1p1s0");

NetworkPort<Network::UdpServer>::NetworkPort(boost::asio::io_context& ioContext, int srcPort, int dstPort, size_t bufferSize_, bool lo) : 
PortManager(ioContext, srcPort, dstPort, bufferSize_)
{
    if (lo)
    {
        m_Lo = std::make_unique<Network::UdpServer>(ioContext, "lo", srcPort, dstPort);
        return;
    }
    m_Eth = std::make_unique<Network::UdpServer>(ioContext, "enP8p1s0", srcPort,  dstPort);
    m_Wlan = std::make_unique<Network::UdpServer>(ioContext, "wlP1p1s0", srcPort, dstPort);
}

NetworkPort<Network::UdpServer>::NetworkPort(boost::asio::io_context& ioContext) : 
PortManager(ioContext, 0, 0)
{
}

NetworkPort<Network::UdpServer>::~NetworkPort()
{
}

int NetworkPort<Network::UdpServer>::open(int srcPort, int dstPort)
{
    if (!m_Eth || !m_Wlan)
    {
        return -1;
    }

    std::string ethAdapter = EthAdpt;
    std::string wlanAdapter = WlanAdpt;

    int ret = false;
    m_EthOk = m_Eth->openSocket(ethAdapter, srcPort, dstPort, bufferSize, false);
    m_WlanOk = m_Wlan->openSocket(wlanAdapter, srcPort, dstPort, bufferSize, false);
    return (m_EthOk && m_WlanOk) ? 0 : -1;
}

int NetworkPort<Network::UdpServer>::close()
{
    int ret = 0;
    if (m_Eth)
    {
        ret = m_Eth->close();
    }
    if (m_Wlan)
    {
        ret = m_Wlan->close();
    }
    return ret;
}

Network::UdpServer* NetworkPort<Network::UdpServer>::eth() const
{
    return m_Eth.get();
}

Network::UdpServer* NetworkPort<Network::UdpServer>::wlan() const
{
    return m_Wlan.get();
}

Network::UdpServer* NetworkPort<Network::UdpServer>::preferred() const
{
    return m_Eth ? m_Eth.get() : m_Wlan.get();
}

bool NetworkPort<Network::UdpServer>::hasEth() const
{
    return static_cast<bool>(m_Eth);
}

bool NetworkPort<Network::UdpServer>::hasWlan() const
{
    return static_cast<bool>(m_Wlan);
}

int NetworkPort<Network::UdpServer>::sendEth(std::string& targetIP, const std::vector<char>& data)
{
    if (targetIP.empty()) return -1;
    
    if (m_Eth)
    {
        m_Eth->transmit(reinterpret_cast<const uint8_t*>(data.data()), data.size(), targetIP);
    }
    else
    {
        return -1; // Ethernet interface not available
    }
    return 0;
}

int NetworkPort<Network::UdpServer>::sendWlan(std::string& targetIP, const std::vector<char>& data)
{
    if (targetIP.empty()) return -1;
    if (m_Wlan)
    {
        m_Wlan->transmit(reinterpret_cast<const uint8_t*>(data.data()), data.size(), targetIP);
    }
    else
    {
        return -1; // WiFi interface not available
    }
    return 0;
}

int NetworkPort<Network::UdpServer>::setReceiveCallback(std::function<void(std::vector<char>&)> callback)
{
    if (m_Eth)
    {
        m_Eth->startReceive(callback);
    }
    if (m_Wlan)
    {
        m_Wlan->startReceive(callback);
    }
    return 0;
}

std::string NetworkPort<Network::UdpServer>::getHostIP()
{
    if (m_Eth)
    {
        return m_Eth->getHostIP();
    }
    if (m_Wlan)
    {
        return m_Wlan->getHostIP();
    }
    return "";
}

NetworkPort<Network::TcpServer>::NetworkPort(boost::asio::io_context& ioContext, int srcPort, int dstPort, size_t bufferSize_, bool lo) : 
PortManager(ioContext, srcPort, dstPort, bufferSize_)
{
    lo_ = lo;
    if (lo_)
    {
        m_Lo = std::make_unique<Network::TcpServer>(ioContext, "lo", "enP8p1s0", srcPort, dstPort);
        return;
    }
    m_Eth = std::make_unique<Network::TcpServer>(ioContext, "enP8p1s0", "wlP1p1s0", srcPort, dstPort);
    m_Wlan = std::make_unique<Network::TcpServer>(ioContext, "wlP1p1s0", "enP8p1s0", srcPort, dstPort);

    // TcpServer's constructor already attempts openSocket() internally; reflect
    // its actual result here so preferred() can tell which interface (if any)
    // really came up, instead of always defaulting to "none".
    m_EthOk = m_Eth->isOpen();
    m_WlanOk = m_Wlan->isOpen();
}

NetworkPort<Network::TcpServer>::NetworkPort(boost::asio::io_context& ioContext) : 
PortManager(ioContext, 0, 0)
{
}

NetworkPort<Network::TcpServer>::~NetworkPort()
{
}

int NetworkPort<Network::TcpServer>::open(int srcPort, int dstPort)
{
    if (m_Lo)
    {
        std::string loAdapter("lo");
        return m_Lo->openSocket(loAdapter, srcPort, dstPort, bufferSize, false) ? 0 : -1;
    }

    if (!m_Eth || !m_Wlan)
    {
        return -1;
    }

    std::string ethAdapter = EthAdpt;
    std::string wlanAdapter = WlanAdpt;

    m_EthOk  = m_Eth->openSocket(ethAdapter, srcPort, dstPort, bufferSize, false);
    m_WlanOk = m_Wlan->openSocket(wlanAdapter, srcPort, dstPort, bufferSize, false);
    return (m_EthOk && m_WlanOk) ? 0 : -1;
}

int NetworkPort<Network::TcpServer>::close()
{
    int ret = 0;
    if (m_Eth)
    {
        ret = m_Eth->close();
    }
    if (m_Wlan)
    {
        ret = m_Wlan->close();
    }
    if (m_Lo)
    {
        ret = m_Lo->close();
    }
    return ret;
}

Network::TcpServer* NetworkPort<Network::TcpServer>::eth() const
{
    return m_Eth.get();
}

Network::TcpServer* NetworkPort<Network::TcpServer>::wlan() const
{
    return m_Wlan.get();
}

Network::TcpServer* NetworkPort<Network::TcpServer>::preferred() const
{
    if (m_Lo)
    {
        return m_Lo.get();
    }
    Network::TcpServer* preferred = nullptr;
    if (m_EthOk && m_WlanOk)
    {
        preferred = (m_Eth) ? m_Eth.get() : m_Wlan.get();
    }
    else if (m_WlanOk && !m_EthOk)
    {
        preferred = m_Wlan.get();
    }
    else if (m_EthOk && !m_WlanOk)
    {
        preferred = m_Eth.get();
    }

    return preferred;
}

bool NetworkPort<Network::TcpServer>::hasEth() const
{
    return static_cast<bool>(m_Eth);
}

bool NetworkPort<Network::TcpServer>::hasWlan() const
{
    return static_cast<bool>(m_Wlan);
}

bool NetworkPort<Network::TcpServer>::hasLo() const
{
    return static_cast<bool>(m_Lo);
}

int NetworkPort<Network::TcpServer>::sendEth(std::string& targetIP, const std::vector<char>& data)
{
    (void)targetIP;
    if (m_Lo)
    {
        return m_Lo->transmit(reinterpret_cast<const uint8_t*>(data.data()), data.size()) ? 0 : -1;
    }

    if (m_Eth)
    {
        m_Eth->transmit(reinterpret_cast<const uint8_t*>(data.data()), data.size());
    }
    else
    {
        return -1; // Ethernet interface not available
    }
    return 0;
}

int NetworkPort<Network::TcpServer>::sendWlan(std::string& targetIP, const std::vector<char>& data)
{
    (void)targetIP;
    if (m_Lo)
    {
        return m_Lo->transmit(reinterpret_cast<const uint8_t*>(data.data()), data.size()) ? 0 : -1;
    }

    if (m_Wlan)
    {
        m_Wlan->transmit(reinterpret_cast<const uint8_t*>(data.data()), data.size());
    }
    else
    {
        return -1; // WiFi interface not available
    }
    return 0;
}

int NetworkPort<Network::TcpServer>::setReceiveCallback(std::function<void(std::vector<char>&)> callback)
{
    if (m_Lo)
    {
        m_Lo->startReceive(callback);
    }

    if (m_Eth)
    {
        m_Eth->startReceive(callback);
    }
    if (m_Wlan)
    {
        m_Wlan->startReceive(callback);
    }
    return 0;
}

std::string NetworkPort<Network::TcpServer>::getHostIP()
{
    if (m_Lo)
    {
        return m_Lo->getHostIP();
    }

    if (m_Eth)
    {
        return m_Eth->getHostIP();
    }
    if (m_Wlan)
    {
        return m_Wlan->getHostIP();
    }
    return "";
}

// TCP Client template specialization would go here if needed
NetworkPort<Network::TcpClient>::NetworkPort(boost::asio::io_context& ioContext, int srcPort, int dstPort, size_t bufferSize_, bool lo) :  
PortManager(ioContext, srcPort, dstPort, bufferSize_)
{
    lo_ = lo;
}

NetworkPort<Network::TcpClient>::NetworkPort(boost::asio::io_context& ioContext) : 
PortManager(ioContext, 0, 0, 1024)
{
}

NetworkPort<Network::TcpClient>::~NetworkPort() = default;

Network::TcpClient* NetworkPort<Network::TcpClient>::eth() const
{
    return m_Eth.get();
}

Network::TcpClient* NetworkPort<Network::TcpClient>::wlan() const
{
    return m_Wlan.get();
}

Network::TcpClient* NetworkPort<Network::TcpClient>::lo() const
{
    return m_Lo.get();
}

Network::TcpClient* NetworkPort<Network::TcpClient>::preferred() const
{
    if (m_Lo)   return m_Lo.get();
    if (m_Eth)  return m_Eth.get();
    if (m_Wlan) return m_Wlan.get();
    return nullptr;
}

bool NetworkPort<Network::TcpClient>::hasEth() const
{
    return static_cast<bool>(m_Eth);
}

bool NetworkPort<Network::TcpClient>::hasWlan() const
{
    return static_cast<bool>(m_Wlan);
}

bool NetworkPort<Network::TcpClient>::hasLo() const
{
    return static_cast<bool>(m_Lo);
}

int NetworkPort<Network::TcpClient>::open(std::string& host, unsigned int dstPort)
{
    if (lo_)
    {
        m_Lo = std::make_unique<Network::TcpClient>(ioContext_, host, 0, dstPort_, bufferSize);
        return 0;
    }
    if (m_Eth)
    {
        m_Eth = std::make_unique<Network::TcpClient>(ioContext_, host, 0, dstPort_, bufferSize);
        return 0;
    }
    if (m_Wlan)
    {
        m_Wlan = std::make_unique<Network::TcpClient>(ioContext_, host, 0, dstPort_, bufferSize);
        return 0;
    }
    return -1;
}

int NetworkPort<Network::TcpClient>::open(int srcPort, int dstPort)
{
    throw std::runtime_error("Open method not implemented for TcpClient NetworkPort.");
}

int NetworkPort<Network::TcpClient>::read(std::vector<char>& buffer)
{
    if (m_Lo)
    {
        return m_Lo->receive(buffer);
    }
    if (m_Eth)
    {
        return m_Eth->receive(buffer);
    }
    if (m_Wlan)
    {
        return m_Wlan->receive(buffer);
    }
    return -1;
}

int NetworkPort<Network::TcpClient>::close()
{
    int result = 0;
    if (m_Lo)
    {
        result = m_Lo->close() ? 0 : -1;
        m_Lo.reset();
        return 0;
    }
    if (m_Eth)
    {
        result = m_Eth->close() ? 0 : -1;
        m_Eth.reset();
    }
    if (m_Wlan)
    {
        result = m_Wlan->close() ? 0 : -1;
        m_Wlan.reset();
    }
    return result;
}

int NetworkPort<Network::TcpClient>::sendEth(std::string& targetIP, const std::vector<char>& data)
{
    (void)targetIP;
    if (m_Lo)
    {
        return m_Lo->transmit(reinterpret_cast<const uint8_t*>(data.data()), data.size()) ? 0 : -1;
    }

    if (m_Eth)
    {
        m_Eth->transmit(reinterpret_cast<const uint8_t*>(data.data()), data.size());
    }
    else
    {
        return -1; // Ethernet interface not available
    }
    return 0;
}

int NetworkPort<Network::TcpClient>::sendWlan(std::string& targetIP, const std::vector<char>& data)
{
    (void)targetIP;
    if (m_Lo)
    {
        return m_Lo->transmit(reinterpret_cast<const uint8_t*>(data.data()), data.size()) ? 0 : -1;
    }

    if (m_Wlan)
    {
        m_Wlan->transmit(reinterpret_cast<const uint8_t*>(data.data()), data.size());
    }
    else
    {
        return -1; // WiFi interface not available
    }
    return 0;
}

int NetworkPort<Network::TcpClient>::setReceiveCallback(std::function<void(std::vector<char>&)> callback)
{
    if (m_Lo)
    {
        m_Lo->startReceive(callback);
        return 0;
    }

    if (m_Eth)
    {
        m_Eth->startReceive(callback);
    }
    if (m_Wlan)
    {
        m_Wlan->startReceive(callback);
    }
    return 0;
}

std::string NetworkPort<Network::TcpClient>::getHostIP()
{
    if (m_Lo)
    {
        return m_Lo->getHostIP();
    }
    if (m_Eth)
    {
        return m_Eth->getHostIP();
    }
    if (m_Wlan)
    {
        return m_Wlan->getHostIP();
    }
    return "";
}

} // namespace NetUtils