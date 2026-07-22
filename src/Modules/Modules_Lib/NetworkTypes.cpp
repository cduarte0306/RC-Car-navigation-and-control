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

int NetworkTcp::receive(std::vector<char>& buffer) {
	if (receiveCallback) {
		return receiveCallback(buffer);
	}
	return -1;
}

int NetworkProxy::
dispatchWebApp(const std::vector<char>& data) {
	if (!sendCallback) {
		return -1;
	}

	ProxyMsgHdr hdr;
	hdr.srcAddr  = NetworkProxy::MainAppRouteAddr;  // Set appropriate source address
	hdr.destAddr = NetworkProxy::WebAppRouteAddr; // Set appropriate destination address
	hdr.len = static_cast<int>(data.size());
	char* tempBuffer = new char[sizeof(ProxyMsgHdr) + data.size()];
	std::memcpy(tempBuffer, &hdr, sizeof(ProxyMsgHdr));
	std::memcpy(tempBuffer + sizeof(ProxyMsgHdr), data.data(), data.size());
	int result = sendCallback(reinterpret_cast<uint8_t*>(tempBuffer), sizeof(ProxyMsgHdr) + data.size());
	delete[] tempBuffer;
	return result;
}

int NetworkProxy::dispatchUpdater(const std::vector<char>& data) {
	if (!sendCallback) {
		return -1;
	}

	ProxyMsgHdr hdr;
	hdr.srcAddr  = NetworkProxy::MainAppRouteAddr;  // Set appropriate source address
	hdr.destAddr = NetworkProxy::UpdaterRouteAddr; // Set appropriate destination address
	hdr.len = static_cast<int>(data.size());
	char* tempBuffer = new char[sizeof(ProxyMsgHdr) + data.size()];
	std::memcpy(tempBuffer, &hdr, sizeof(ProxyMsgHdr));
	std::memcpy(tempBuffer + sizeof(ProxyMsgHdr), data.data(), data.size());
	int result = sendCallback(reinterpret_cast<uint8_t*>(tempBuffer), sizeof(ProxyMsgHdr) + data.size());
	delete[] tempBuffer;
	return result;
}

int NetworkProxy::routeMsg(const std::vector<char>& data) {
	if (data.size() < sizeof(NetworkProxy::ProxyMsgHdr))
		return 0;
	
	const ProxyMsgHdr* hdr = reinterpret_cast<const ProxyMsgHdr*>(data.data());
	auto callback = routeCallbacks[hdr->destAddr];
	if (callback) {
		return callback(data);
	}
	return -1;
}

template <typename T>
int NetworkProxy::registerCallbacks(int (T::*webAppCallback)(const std::vector<char>&),
                                    int (T::*updaterCallback)(const std::vector<char>&)) {
    this->webAppCallback = [webAppCallback](const std::vector<char>& data) {
        return (static_cast<T*>(nullptr)->*webAppCallback)(data);
    };
    this->updaterCallback = [updaterCallback](const std::vector<char>& data) {
        return (static_cast<T*>(nullptr)->*updaterCallback)(data);
    };
    return 0;
}