#include "Modules_Lib/AdapterBase.hpp"

namespace Adapter {

std::string CommsAdapter::readStats()
{
	if (readStatsCommand)
	{
		return readStatsCommand();
	}
	return "";
}

CommsAdapter::CommsAdapter(std::string parentName_) : AdapterBase(ModuleDefs::AdapterId::CommsAdapterID, parentName_)
{
}

int CommsAdapter::transmitData(const uint8_t* data, size_t length)
{
	if (!data || length == 0)
	{
		return -1;
	}

	return transmitDataCommand(data, length);
}

int CommsAdapter::startReceive(NetworkAdapter& adapter, std::function<void(std::vector<char>&)> callback, bool asyncTx)
{
	if (!callback)
	{
		return -1;
	}

	dataReceivedCommand(adapter, callback, asyncTx);
	return 0;
}

int CommsAdapter::startReceive(NetworkAdapter& adapter)
{
	if (!OnModuleMsgReceivedFunc)
	{
		return -1;
	}
	dataReceivedCommand(adapter, [this](std::vector<char>& buffer)
	{
		return OnModuleMsgReceivedFunc(buffer);
	}, true);
	return 0;
}


int CommsAdapter::openAdapter(int port, std::string& adapter)
{
	(void)port;
	(void)adapter;
	return 0;
}

template<typename T, int BufferSize>
std::unique_ptr<T> CommsAdapter::OpenNetworkAdapter(const std::string& callerName, int sPort, int dPort, bool internal, bool broadcast)
{
	static_assert(std::is_base_of<NetworkAdapter, T>::value, "createAdapter: T must derive from NetworkAdapter");

	int adapterType = -1;
	if constexpr (std::is_same<T, NetworkUdp>::value)
	{
		adapterType = CommsAdapter::UdpAdapterType;
	}
	else if constexpr (std::is_same<T, NetworkTcpServer>::value)
	{
		adapterType = CommsAdapter::TcpServerAdapterType;
	}
	else if constexpr (std::is_same<T, NetworkTcpClient>::value)
	{
		adapterType = CommsAdapter::TcpClientAdapterType;
	}
	else if constexpr (std::is_same<T, NetworkProxy>::value)
	{
		adapterType = CommsAdapter::TcpProxyAdapterType;
	}
	else
	{
		static_assert(!std::is_same<T, T>::value, "Unsupported type for OpenNetworkAdapter");
	}

	auto baseAdapter = openAdapterCommand(callerName, adapterType, sPort, dPort, BufferSize, broadcast, internal);
	if (!baseAdapter)
	{
		return nullptr;
	}

	return std::unique_ptr<T>(static_cast<T*>(baseAdapter.release()));
}

bool CommsAdapter::GetEthConnectionState() const
{
	return ethConnectionState.load();
}

void CommsAdapter::RefreshConnectionState()
{
	if (refreshConnectionStateCommand)
	{
		refreshConnectionStateCommand();
	}
}

int CommsAdapter::bind_(AdapterBase* Adapter)
{
	CommsAdapter* Adapter_ = static_cast<CommsAdapter*>(Adapter);
	bindInterface(Adapter_);
	return 0;
}

void CommsAdapter::bindInterface(CommsAdapter* adapter)
{
	if (!adapter)
	{
		return;
	}

	this->transmitDataCommand = [adapter](const uint8_t* pData, size_t length) -> int
	{
		return adapter->transmitData_(pData, length);
	};

	this->openAdapterCommand = [adapter](const std::string& parent, int type, int sPort, int dPort, size_t bufferSize, bool broadcast, bool loopback) -> std::unique_ptr<NetworkAdapter>
	{
		return adapter->openAdapter_(parent, type, sPort, dPort, bufferSize, broadcast, loopback);
	};

	this->dataReceivedCommand = [adapter](NetworkAdapter& netAdp, std::function<void(std::vector<char>&)> callback, bool asyncTx) -> int
	{
		adapter->configureReceiveCallback(netAdp, callback, asyncTx);
		return 0;
	};

	this->clientConnectedCommand = [adapter](NetworkAdapter& netAdp, std::function<void(std::string&)> callback) -> int
	{
		adapter->configureOnConnectCallback(netAdp, callback);
		return 0;
	};

	this->readStatsCommand = [adapter]() -> std::string
	{
		return adapter->readStats();
	};

	this->refreshConnectionStateCommand = [adapter]() -> void
	{
		adapter->RefreshConnectionState_();
	};
}

void CommsAdapter::startReceive_(NetworkAdapter& adapter, std::function<void(std::vector<char>&)> dataReceivedCommand_, bool asyncTx)
{
	(void)adapter;
	(void)dataReceivedCommand_;
	(void)asyncTx;
}

void CommsAdapter::configureReceiveCallback(NetworkAdapter& adapter, std::function<void(std::vector<char>&)> callback, bool asyncTx)
{
	(void)adapter;
	(void)callback;
	(void)asyncTx;
}

void CommsAdapter::configureOnConnectCallback(NetworkAdapter& adapter, std::function<void(std::string& hostIp)> callback)
{
	(void)adapter;
	(void)callback;
}

int CommsAdapter::transmitData_(const uint8_t* data, size_t length)
{
	if (!data || length == 0)
	{
		return -1;
	}

	return 0;
}

std::unique_ptr<NetworkAdapter> CommsAdapter::openAdapter_(const std::string& parent, int type, int sPort, int dPort, size_t bufferSize, bool broadcast, bool loopback)
{
	std::string adapterDesc(parent);
	m_RegisteredCallers.push_back(parent);

	auto it = adapterMap.find(parent);
	if (it != adapterMap.end() && it->second)
	{
		CommsAdapter* boundAdapter = reinterpret_cast<CommsAdapter*>(it->second);
		m_CallerAdapterMap[parent] = boundAdapter;
	}

	std::unique_ptr<NetworkAdapter> netAdapter;
	switch (type)
	{
		case CommsAdapter::UdpAdapterType:
			netAdapter = std::make_unique<NetworkUdp>(adapterDesc, sPort, dPort, bufferSize);
			break;
		case CommsAdapter::TcpServerAdapterType:
			netAdapter = std::make_unique<NetworkTcpServer>(adapterDesc, sPort, dPort, bufferSize);
			break;
		case CommsAdapter::TcpClientAdapterType:
			netAdapter = std::make_unique<NetworkTcpClient>(adapterDesc, sPort, dPort, bufferSize);
			break;
		case CommsAdapter::TcpProxyAdapterType:
		{
			const int proxyPort = (dPort > 0) ? dPort : sPort;
			netAdapter = std::make_unique<NetworkProxy>(proxyPort, bufferSize);
			break;
		}
		default:
			return nullptr;
	}

	if (!netAdapter)
	{
		return nullptr;
	}

	adapterCounter++;
	netAdapter->id = adapterCounter;
	netAdapter->setParent(parent);
	netAdapter->broadcast = broadcast;
	netAdapter->loopback = loopback;

	const int cfgStatus = configureAdapter(*netAdapter, netAdapter->id, type, loopback);
	if (cfgStatus != 0)
	{
		std::string msg("Failed to configure adapter " + adapterDesc + " for parent " + parent + "\n");
		(void)msg;
	}

	return netAdapter;
}

int CommsAdapter::configureAdapter(NetworkAdapter& netAdapter, int adapterIdx, int type, bool internal)
{
	(void)netAdapter;
	(void)adapterIdx;
	(void)type;
	(void)internal;
	return 0;
}

void CommsAdapter::RefreshConnectionState_() {}

template std::unique_ptr<NetworkUdp> CommsAdapter::OpenNetworkAdapter<NetworkUdp, 2048>(const std::string& callerName, int sPort, int dPort, bool internal, bool broadcast);
template std::unique_ptr<NetworkUdp> CommsAdapter::OpenNetworkAdapter<NetworkUdp, CommsAdapter::MaxUDPPacketSize>(const std::string& callerName, int sPort, int dPort, bool internal, bool broadcast);
template std::unique_ptr<NetworkTcpServer> CommsAdapter::OpenNetworkAdapter<NetworkTcpServer, 2048>(const std::string& callerName, int sPort, int dPort, bool internal, bool broadcast);
template std::unique_ptr<NetworkTcpServer> CommsAdapter::OpenNetworkAdapter<NetworkTcpServer, CommsAdapter::MaxUDPPacketSize>(const std::string& callerName, int sPort, int dPort, bool internal, bool broadcast);
template std::unique_ptr<NetworkProxy> CommsAdapter::OpenNetworkAdapter<NetworkProxy, 2048>(const std::string& callerName, int sPort, int dPort, bool internal, bool broadcast);

} // namespace Adapter