#include "NetworkTypes.hpp"
#include "AdapterBase.hpp"

using namespace Adapter;

NetworkAdapter::NetworkAdapter(const std::string& adapter_, int sPort_, int dPort_, size_t bufferSize_)
	: sPort(sPort_), dPort(dPort_), bufferSize(bufferSize_), adapter(adapter_) {
}

NetworkAdapter::~NetworkAdapter() {
}

int NetworkAdapter::send(const uint8_t* data, size_t length, std::string destIp) {
	if (sendCallbackTcp) {
		return sendCallbackTcp(data, length);
	} else if (sendCallback) {
		return sendCallback(data, length);
	}
	return -1;
}

int NetworkTcp::receive(std::vector<char>& buffer) {
	if (receiveCallback) {
		return receiveCallback(buffer);
	}
	return -1;
}

int NetworkAdapter::getPreferredSrcPort() const {
	if (preferredSrcPortCb) {
		const int port = preferredSrcPortCb();
		if (port > 0) {
			return port;
		}
	}

	return sPort;
}

int NetworkAdapter::closeSocket() {
	if (closeSocketCb) {
		return closeSocketCb();
	}

	connected = false;
	return 0;
}

bool NetworkAdapter::IsEthPresent(void) const {
	if (!EthPresent) return false;
	return EthPresent();
}

bool NetworkAdapter::IsHostPresent(void) const {
	if (!hostPresentCB) return false;
	return hostPresentCB();
}

void NetworkAdapter::setParent(const std::string& name) {
	parent = name;
}

void NetworkAdapter::OnEthLinkDetected(bool state) {
	ethLinkDetected.store(state);
}

void NetworkAdapter::OnWlanLinkDetected(bool state) {
	wlanLinkDetected.store(state);
}

namespace Adapter {

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

std::unique_ptr<NetworkAdapter> CommsAdapter::OpenNetworkAdapter(const std::string& callerName, uint8_t type, int sPort, int dPort, std::string adapter, size_t bufferSize, bool broadcast) {
	return openAdapterCommand(callerName, type, sPort, dPort, bufferSize, broadcast, false);
}

std::unique_ptr<NetworkAdapter> CommsAdapter::OpenNetworkLoopbackAdapter(const std::string& callerName, uint8_t type, int sPort, int dPort, size_t bufferSize, bool broadcast) {
	return openAdapterCommand(callerName, type, sPort, dPort, bufferSize, broadcast, true);
}

} // namespace Adapter
