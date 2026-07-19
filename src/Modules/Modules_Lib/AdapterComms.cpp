#include "Modules_Lib/AdapterBase.hpp"

namespace Adapter {

CommsAdapter::NetworkAdapter::NetworkAdapter(const std::string& adapter_, int sPort_, int dPort_, size_t bufferSize_)
	: sPort(sPort_), dPort(dPort_), bufferSize(bufferSize_), adapter(adapter_) {
}

CommsAdapter::NetworkAdapter::~NetworkAdapter() {
}

int CommsAdapter::NetworkAdapter::send(const uint8_t* data, size_t length, std::string destIp) {
	if (sendCallbackTcp) {
		return sendCallbackTcp(data, length);
	} else if (sendCallback) {
		return sendCallback(data, length);
	}
	return -1;
}

int CommsAdapter::NetworkTcp::receive(std::vector<char>& buffer) {
	if (receiveCallback) {
		return receiveCallback(buffer);
	}
	return -1;
}

int CommsAdapter::NetworkAdapter::getPreferredSrcPort() const {
	if (preferredSrcPortCb) {
		const int port = preferredSrcPortCb();
		if (port > 0) {
			return port;
		}
	}

	return sPort;
}

int CommsAdapter::NetworkAdapter::closeSocket() {
	if (closeSocketCb) {
		return closeSocketCb();
	}

	connected = false;
	return 0;
}

bool CommsAdapter::NetworkAdapter::IsEthPresent(void) const {
	if (!EthPresent) return false;
	return EthPresent();
}

bool CommsAdapter::NetworkAdapter::IsHostPresent(void) const {
	if (!hostPresentCB) return false;
	return hostPresentCB();
}

void CommsAdapter::NetworkAdapter::setParent(const std::string& name) {
	parent = name;
}

void CommsAdapter::NetworkAdapter::OnEthLinkDetected(bool state) {
	ethLinkDetected.store(state);
}

void CommsAdapter::NetworkAdapter::OnWlanLinkDetected(bool state) {
	wlanLinkDetected.store(state);
}

CommsAdapter::CommsAdapter(std::string parentName_) : AdapterBase(ModuleDefs::AdapterId::CommsAdapterID, parentName_) {
}

int CommsAdapter::transmitData(const uint8_t* data, size_t length) {
	if (!data || length == 0) {
		return -1;
	}

	return transmitDataCommand(data, length);
}

int CommsAdapter::startReceive(NetworkAdapter& adapter, std::function<void(std::vector<char>&)> callback, bool asyncTx) {
	if (!callback) {
		return -1;
	}

	dataReceivedCommand(adapter, callback, asyncTx);
	return 0;
}

int CommsAdapter::startReceive(NetworkAdapter& adapter) {
	if (!OnModuleMsgReceivedFunc) {
		return -1;
	}
	dataReceivedCommand(adapter, [this](std::vector<char>& buffer) {
		return OnModuleMsgReceivedFunc(buffer);
	}, true);
	return 0;
}


int CommsAdapter::openAdapter(int port, std::string& adapter) {
	(void)port;
	(void)adapter;
	return 0;
}

std::unique_ptr<CommsAdapter::NetworkAdapter> CommsAdapter::OpenNetworkAdapter(const std::string& callerName, uint8_t type, int sPort, int dPort, std::string adapter, size_t bufferSize, bool broadcast) {
	return openAdapterCommand(callerName, type, sPort, dPort, bufferSize, broadcast, false);
}

std::unique_ptr<CommsAdapter::NetworkAdapter> CommsAdapter::OpenNetworkLoopbackAdapter(const std::string& callerName, uint8_t type, int sPort, int dPort, size_t bufferSize, bool broadcast) {
	return openAdapterCommand(callerName, type, sPort, dPort, bufferSize, broadcast, true);
}

std::string CommsAdapter::readStats() {
	if (readStatsCommand) {
		return readStatsCommand();
	}
	return "";
}

bool CommsAdapter::GetEthConnectionState() const {
	return ethConnectionState.load();
}

int CommsAdapter::bind_(AdapterBase* Adapter) {
	CommsAdapter* Adapter_ = static_cast<CommsAdapter*>(Adapter);
	bindInterface(Adapter_);
	return 0;
}

void CommsAdapter::bindInterface(CommsAdapter* adapter) {
	if (!adapter) {
		return;
	}

	this->transmitDataCommand = [adapter](const uint8_t* pData, size_t length) -> int {
		return adapter->transmitData_(pData, length);
	};

	this->openAdapterCommand = [adapter](const std::string& parent, int type, int sPort, int dPort, size_t bufferSize, bool broadcast, bool loopback) -> std::unique_ptr<NetworkAdapter> {
		return adapter->openAdapter_(parent, type, sPort, dPort, bufferSize, broadcast, loopback);
	};

	this->dataReceivedCommand = [adapter](NetworkAdapter& netAdp, std::function<void(std::vector<char>&)> callback, bool asyncTx) -> int {
		adapter->configureReceiveCallback(netAdp, callback, asyncTx);
		return 0;
	};

	this->clientConnectedCommand = [adapter](NetworkAdapter& netAdp, std::function<void(std::string&)> callback) -> int {
		adapter->configureOnConnectCallback(netAdp, callback);
		return 0;
	};

	this->readStatsCommand = [adapter]() -> std::string {
		return adapter->readStats();
	};
}

void CommsAdapter::startReceive_(NetworkAdapter& adapter, std::function<void(std::vector<char>&)> dataReceivedCommand_, bool asyncTx) {
	(void)adapter;
	(void)dataReceivedCommand_;
	(void)asyncTx;
}

void CommsAdapter::configureReceiveCallback(NetworkAdapter& adapter, std::function<void(std::vector<char>&)> callback, bool asyncTx) {
	(void)adapter;
	(void)callback;
	(void)asyncTx;
}

void CommsAdapter::configureOnConnectCallback(NetworkAdapter& adapter, std::function<void(std::string& hostIp)> callback) {
	(void)adapter;
	(void)callback;
}

int CommsAdapter::transmitData_(const uint8_t* data, size_t length) {
	if (!data || length == 0) {
		return -1;
	}

	return 0;
}

std::unique_ptr<CommsAdapter::NetworkAdapter> CommsAdapter::openAdapter_(const std::string& parent, int type, int sPort, int dPort, size_t bufferSize, bool broadcast, bool loopback) {
	std::string adapterDesc(parent);
	m_RegisteredCallers.push_back(parent);

	auto it = adapterMap.find(parent);
	if (it != adapterMap.end() && it->second) {
		CommsAdapter* boundAdapter = reinterpret_cast<CommsAdapter*>(it->second);
		m_CallerAdapterMap[parent] = boundAdapter;
	}

	std::unique_ptr<NetworkAdapter> netAdapter = std::make_unique<NetworkAdapter>(adapterDesc, sPort, dPort, bufferSize);
	adapterCounter++;
	netAdapter->id = adapterCounter;
	netAdapter->setParent(parent);
	netAdapter->broadcast = broadcast;
	netAdapter->loopback = loopback;

	const int cfgStatus = configureAdapter(*netAdapter, netAdapter->id, type, loopback);
	if (cfgStatus != 0) {
		std::string msg("Failed to configure adapter " + adapterDesc + " for parent " + parent + "\n");
		(void)msg;
	}

	return netAdapter;
}

int CommsAdapter::configureAdapter(NetworkAdapter& netAdapter, int adapterIdx, int type, bool internal) {
	(void)netAdapter;
	(void)adapterIdx;
	(void)type;
	(void)internal;
	return 0;
}

} // namespace Adapter