#include "NetworkTypes.hpp"
#include "AdapterBase.hpp"

using namespace Adapter;

NetworkAdapter::NetworkAdapter(const std::string& adapter_, int sPort_, int dPort_, size_t bufferSize_)
	: sPort(sPort_), dPort(dPort_), bufferSize(bufferSize_), adapter(adapter_)
{
}

NetworkAdapter::~NetworkAdapter()
{
}

int NetworkAdapter::send(const uint8_t* data, size_t length, std::string destIp)
{
	if (sendCallbackTcp)
	{
		return sendCallbackTcp(data, length);
	}
	else if (sendCallback)
	{
		return sendCallback(data, length);
	}
	return -1;
}

int NetworkAdapter::getPreferredSrcPort() const
{
	if (preferredSrcPortCb)
	{
		const int port = preferredSrcPortCb();
		if (port > 0)
		{
			return port;
		}
	}

	return sPort;
}

int NetworkAdapter::closeSocket()
{
	if (closeSocketCb)
	{
		return closeSocketCb();
	}

	connected = false;
	return 0;
}

bool NetworkAdapter::IsEthPresent(void) const
{
	if (!EthPresent) return false;
	return EthPresent();
}

bool NetworkAdapter::IsHostPresent(void) const
{
	if (!hostPresentCB) return false;
	return hostPresentCB();
}

void NetworkAdapter::setParent(const std::string& name)
{
	parent = name;
}

void NetworkAdapter::OnEthLinkDetected(bool state)

{
	ethLinkDetected.store(state);
}

void NetworkAdapter::OnWlanLinkDetected(bool state)
{
	wlanLinkDetected.store(state);
}

int NetworkTcpServer::receive(std::vector<char>& buffer)
{
	if (receiveCallback)
	{
		return receiveCallback(buffer);
	}
	return -1;
}

int NetworkTcpClient::receive(std::vector<char>& buffer)
{
	if (receiveCallback)
	{
		return receiveCallback(buffer);
	}
	return -1;
}

int NetworkProxy::dispatchWebApp(const nlohmann::json& msg)
{
	if (!sendCallback)
	{
		return -1;
	}

	// ProxyMsgHdr hdr;
	// hdr.srcAddr  = NetworkProxy::MainAppRouteAddr;  // Set appropriate source address
	// hdr.destAddr = NetworkProxy::WebAppRouteAddr; // Set appropriate destination address
	// hdr.len = static_cast<int>(msg.dump().length());
	size_t msglen = sizeof(ProxyMsgHdr) + msg.dump().length();
	char* tempBuffer = new char[msglen];
	std::memset(tempBuffer, 0, msglen);
	// std::memcpy(tempBuffer, &hdr, sizeof(ProxyMsgHdr));
	ProxyMsgHdr* hdr = reinterpret_cast<ProxyMsgHdr*>(tempBuffer);
	hdr->srcAddr  = NetworkProxy::MainAppRouteAddr;  // Set appropriate source address
	hdr->destAddr = NetworkProxy::WebAppRouteAddr; // Set appropriate destination address
	hdr->len = static_cast<int>(msg.dump().length());
	std::memcpy(tempBuffer + sizeof(ProxyMsgHdr), msg.dump().data(), msg.dump().length());
	int result = sendCallback(reinterpret_cast<uint8_t*>(tempBuffer), msglen);
	delete[] tempBuffer;
	return result;
}

int NetworkProxy::dispatchUpdater(const nlohmann::json& msg)
{
	if (!sendCallback)
	{
		return -1;
	}

	size_t msglen = sizeof(ProxyMsgHdr) + msg.dump().length();
	char* tempBuffer = new char[msglen];
	std::memset(tempBuffer, 0, msglen);
	// std::memcpy(tempBuffer, &hdr, sizeof(ProxyMsgHdr));
	ProxyMsgHdr* hdr = reinterpret_cast<ProxyMsgHdr*>(tempBuffer);
	hdr->srcAddr  = NetworkProxy::MainAppRouteAddr;  // Set appropriate source address
	hdr->destAddr = NetworkProxy::UpdaterRouteAddr; // Set appropriate destination address
	hdr->len = static_cast<int>(msg.dump().length());
	std::memcpy(tempBuffer + sizeof(ProxyMsgHdr), msg.dump().data(), msg.dump().length());
	int result = sendCallback(reinterpret_cast<uint8_t*>(tempBuffer), msglen);
	delete[] tempBuffer;
	return result;
}

int NetworkProxy::routeMsg(const std::vector<char>& data)
{
	if (data.size() < sizeof(NetworkProxy::ProxyMsgHdr))
		return 0;
	
	const ProxyMsgHdr* hdr = reinterpret_cast<const ProxyMsgHdr*>(data.data());
	if (hdr->destAddr < 0 || static_cast<size_t>(hdr->destAddr) == MaxRouteAddr)
		return -1;
	auto callback = routeCallbacks[hdr->srcAddr];
	if (callback)
	{
		try
		{
			nlohmann::json jsonData = nlohmann::json::parse(std::string(data.data() + sizeof(ProxyMsgHdr), hdr->len));
			return callback(jsonData);
		}
		catch (...)
		{
			return -1;
		}
	}
	return -1;
}
