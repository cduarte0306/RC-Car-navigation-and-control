#include "AdapterBase.hpp"
#include "lib/RegisterMap.hpp"

namespace Adapter {

AdapterBase::AdapterBase(ModuleDefs::AdapterId id, std::string parentName_) : parentName(parentName_), adapterId(id) {
	// Resolve the parent module's ID
	RegisterMap* regMap = RegisterMap::getInstance();
	if (regMap) {
		if (auto moduleMap = regMap->get<std::unordered_map<std::string, int>>(RegisterMap::RegisterKeys::ModuleMap)) {
			auto it = moduleMap->find(parentName_);
			if (it != moduleMap->end()) {
				m_ModuleID = it->second;
			}
		}
	}
}

AdapterBase::~AdapterBase() {
}

int AdapterBase::bind(AdapterBase* Adapter) {
	adapterMap[Adapter->getParentName()] = Adapter;

	this->moduleWriteCmd = [Adapter](char* pbuf, size_t len) {
		return Adapter->moduleCommand_(pbuf, len);
	};

	this->moduleWriteCmdVector = [Adapter](std::vector<char>& buffer) {
		return Adapter->moduleCommand_(buffer);
	};

	return bind_(Adapter);
}

int AdapterBase::bindCommandDispatch(std::function<int(Msg::MessageCapsule<char>& capsule)> dispatchFunc) {
	if (!dispatchFunc) {
		return -1;
	}

	this->dispatchCommandFunc = dispatchFunc;
	return 0;
}

int AdapterBase::bindOnModuleMsgReceived(std::function<int(std::vector<char>& buffer)> onMsgReceivedFunc) {
	if (!onMsgReceivedFunc) {
		return -1;
	}

	this->OnModuleMsgReceivedFunc = onMsgReceivedFunc;
	return 0;
}

std::string AdapterBase::readStats() {
	if (!readStatsCommand) {
		return "";
	}

	return readStatsCommand();
}

int AdapterBase::stopCmd(void) {
	return 0;
}

int AdapterBase::dispatchCommand(Msg::MessageCapsule<char>& capsule) {
	if (!dispatchCommandFunc) {
		return -1;
	}

	dispatchCommandFunc(capsule);
	return 0;
}

int AdapterBase::moduleCommand(char* pbuf, size_t len) {
	if (!moduleWriteCmd) {
		std::cout << "Module command error\r\n";
		return -1;
	}

	moduleWriteCmd(pbuf, len);
	return 0;
}

int AdapterBase::moduleCommand(std::vector<char>& buffer) {
	if (!moduleWriteCmd) {
		std::cout << "Module command error\r\n";
		return -1;
	}

	return moduleWriteCmdVector(buffer);
}

int AdapterBase::cliCommand(std::vector<std::string>& buffer) {
	if (!moduleCliCmd) {
		std::cout << "Module CLI command error\r\n";
		return -1;
	}

	return moduleCliCmd(buffer);
}

void AdapterBase::addAdapter(std::string moduleName) {
	boundModules.push_back(moduleName);
}

std::string AdapterBase::getParentName() const {
	return parentName;
}

int AdapterBase::moduleCommand_(char* pbuf, size_t len) {
	(void)pbuf;
	(void)len;
	return -1;
}

int AdapterBase::moduleCommand_(std::vector<char>& buffer) {
	(void)buffer;
	return -1;
}

int AdapterBase::moduleCommandAsync_(char* pbuf, size_t len) {
	(void)pbuf;
	(void)len;
	return -1;
}

int AdapterBase::moduleCliCmd_(std::vector<std::string>& buffer) {
	(void)buffer;
	return -1;
}

MotorAdapter::MotorAdapter(std::string parentName_) : AdapterBase(ModuleDefs::AdapterId::MotorAdapterID, parentName_) {
}

int MotorAdapter::setMotorSpeed(int speed) {
	if (!motorSpeedCommand) {
		return -1;
	}

	return motorSpeedCommand(speed);
}

int MotorAdapter::steer(int angle) {
	if (!steerCommand) {
		return -1;
	}

	steerCommand(angle);
	return 0;
}

int MotorAdapter::CommandMotorState(bool state) {
	if (!disableMotorsCommand || !enableMotorsCommand) {
		return 0;
	}

	if (state) {
		enableMotorsCommand();
	} else {
		disableMotorsCommand();
	}
	return 0;
}

int MotorAdapter::bind_(AdapterBase* Adapter) {
	bindInterface(static_cast<MotorAdapter*>(Adapter));
	return 0;
}

void MotorAdapter::bindInterface(MotorAdapter* adapter) {
	if (!adapter) {
		return;
	}

	this->motorSpeedCommand = [adapter](int speed) {
		return adapter->setMotorSpeed_(speed);
	};

	this->steerCommand = [adapter](int angle) {
		return adapter->steer_(angle);
	};

	this->moduleWriteCmd = [adapter](char* pbuf, size_t len) {
		return adapter->moduleCommand_(pbuf, len);
	};
}

int MotorAdapter::setMotorSpeed_(int direction) {
	(void)direction;
	return 0;
}

int MotorAdapter::steer_(int counts) {
	(void)counts;
	return 0;
}

CLIAdapter::CLIAdapter(std::string parentName_) : AdapterBase(ModuleDefs::AdapterId::CliAdapterID, parentName_) {
}

int CLIAdapter::bind_(AdapterBase* Adapter) {
	(void)Adapter;
	return 0;
}

void CLIAdapter::bindInterface(CLIAdapter* adapter) {
	if (!adapter) {
		return;
	}

	this->readStats = [adapter]() -> std::string {
		return adapter->readModuleStats_();
	};
}

std::string CLIAdapter::readModuleStats_(void) {
	return "";
}

CameraAdapter::CameraAdapter(std::string parentName_) : AdapterBase(ModuleDefs::AdapterId::CameraAdapterID, parentName_) {
}

int CameraAdapter::setCameraState(bool state) {
	(void)state;
	return 0;
}

std::string CameraAdapter::readStats() {
	if (readStatsCommand) {
		return readStatsCommand();
	}
	return "";
}

int CameraAdapter::bind_(AdapterBase* Adapter) {
	bindInterface(static_cast<CameraAdapter*>(Adapter));
	return 0;
}

void CameraAdapter::bindInterface(CameraAdapter* adapter) {
	if (!adapter) {
		return;
	}

	this->setCameraStateCommand = [adapter](bool state) -> int {
		return adapter->setCameraState_(state);
	};

	this->moduleWriteCmd = [adapter](char* pbuf, size_t len) {
		return adapter->moduleCommand_(pbuf, len);
	};

	this->moduleWriteCmdVector = [adapter](std::vector<char>& buffer) {
		return adapter->moduleCommand_(buffer);
	};

	this->moduleCliCmd = [adapter](std::vector<std::string>& buffer) {
		return adapter->moduleCliCmd_(buffer);
	};

	this->readStatsCommand = [adapter]() -> std::string {
		return adapter->readStats();
	};
}

int CameraAdapter::setCameraState_(int direction) {
	(void)direction;
	return 0;
}

int CameraAdapter::configurePipeline_(const std::string& host) {
	(void)host;
	return 0;
}

UpdateAdapter::UpdateAdapter(std::string parentName_) : AdapterBase(ModuleDefs::AdapterId::UpdateAdapterID, parentName_) {
}

int UpdateAdapter::bind_(AdapterBase* Adapter) {
	bindInterface(static_cast<UpdateAdapter*>(Adapter));
	return 0;
}

void UpdateAdapter::bindInterface(UpdateAdapter* adapter) {
	if (!adapter) {
		return;
	}

	this->moduleWriteCmd = [adapter](char* pbuf, size_t len) {
		return adapter->moduleCommand_(pbuf, len);
	};

	this->moduleWriteCmdVector = [adapter](std::vector<char>& buffer) {
		return adapter->moduleCommand_(buffer);
	};
}

CommsAdapter::NetworkAdapter::NetworkAdapter(const std::string& adapter_, int sPort_, int dPort_, size_t bufferSize_)
	: adapter(adapter_), sPort(sPort_), dPort(dPort_), bufferSize(bufferSize_) {
}

CommsAdapter::NetworkAdapter::~NetworkAdapter() {
}

int CommsAdapter::NetworkAdapter::send(const uint8_t* data, size_t length, std::string destIp) {
	if (sendCallbackTcp) {
		return sendCallbackTcp(data, length);
	} else if (sendCallback) {
		return sendCallback(destIp, data, length);
	}
	return -1;
}

void CommsAdapter::NetworkAdapter::setParent(const std::string& name) {
	parent = name;
}

std::string CommsAdapter::NetworkAdapter::getHostIP() const {
	if (hostResolver) {
		return hostResolver();
	}
	return std::string();
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

std::string CommsAdapter::getHostIP(NetworkAdapter& adapter) {
	if (!hostIPQueryCommand) {
		return std::string();
	}
	return hostIPQueryCommand(adapter);
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

std::unique_ptr<CommsAdapter::NetworkAdapter> CommsAdapter::createNetworkAdapter(const std::string& callerName, uint8_t type, int sPort, int dPort, std::string adapter, size_t bufferSize, bool broadcast) {
	if (type == UdpAdapterType) {
		return openAdapterCommand(callerName, sPort, dPort, adapter, bufferSize, broadcast);
	} else if (type == TcpAdapterType) {
		return openTcpAdapterCommand(callerName, sPort, dPort, adapter, bufferSize, broadcast);
	}
	return nullptr;
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

	this->openAdapterCommand = [adapter](const std::string& parent, int sPort, int dPort, const std::string& adpName, size_t bufferSize, bool broadcast) -> std::unique_ptr<NetworkAdapter> {
		return adapter->openAdapter_(parent, sPort, dPort, adpName, bufferSize, broadcast);
	};

	this->openTcpAdapterCommand = [adapter](const std::string& parent, int sPort, int dPort, const std::string& adpName, size_t bufferSize, bool broadcast) -> std::unique_ptr<NetworkAdapter> {
		return adapter->openTcpAdapter_(parent, sPort, dPort, adpName, bufferSize, broadcast);
	};

	this->dataReceivedCommand = [adapter](NetworkAdapter& netAdp, std::function<void(std::vector<char>&)> callback, bool asyncTx) -> int {
		adapter->configureReceiveCallback(netAdp, callback, asyncTx);
		return 0;
	};

	this->hostIPQueryCommand = [adapter](NetworkAdapter& netAdp) -> std::string {
		return adapter->getHostIP_(netAdp);
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

std::string CommsAdapter::getHostIP_(NetworkAdapter& adapter) {
	return adapter.getHostIP();
}

int CommsAdapter::transmitData_(const uint8_t* data, size_t length) {
	if (!data || length == 0) {
		return -1;
	}

	return 0;
}

std::unique_ptr<CommsAdapter::NetworkAdapter> CommsAdapter::openAdapter_(const std::string& parent, int sPort, int dPort, const std::string& adapter, size_t bufferSize, bool broadcast) {
	std::pair<std::string, std::string> adapterDesc(parent, adapter);
	m_RegisteredCallers.push_back(adapterDesc);

	auto it = adapterMap.find(parent);
	if (it != adapterMap.end() && it->second) {
		CommsAdapter* boundAdapter = reinterpret_cast<CommsAdapter*>(it->second);
		m_CallerAdapterMap[parent] = boundAdapter;
	}

	std::unique_ptr<NetworkAdapter> netAdapter = std::make_unique<NetworkAdapter>(adapter, sPort, dPort, bufferSize);
	adapterCounter++;
	netAdapter->id = adapterCounter;
	netAdapter->setParent(parent);
	netAdapter->broadcast = broadcast;

	const int cfgStatus = configureUDPAdapter(*netAdapter, netAdapter->id);
	if (cfgStatus != 0) {
		std::string msg("Failed to configure adapter " + adapter + " for parent " + parent + "\n");
		(void)msg;
	}

	return netAdapter;
}

std::unique_ptr<CommsAdapter::NetworkAdapter> CommsAdapter::openTcpAdapter_(const std::string& parent, int sPort, int dPort, const std::string& adapter, size_t bufferSize, bool broadcast) {
	std::pair<std::string, std::string> adapterDesc(parent, adapter);
	m_RegisteredCallers.push_back(adapterDesc);

	auto it = adapterMap.find(parent);
	if (it != adapterMap.end() && it->second) {
		CommsAdapter* boundAdapter = reinterpret_cast<CommsAdapter*>(it->second);
		m_CallerAdapterMap[parent] = boundAdapter;
	}

	std::unique_ptr<NetworkAdapter> netAdapter = std::make_unique<NetworkAdapter>(adapter, sPort, dPort, bufferSize);
	adapterCounter++;
	netAdapter->id = adapterCounter;
	netAdapter->setParent(parent);
	netAdapter->broadcast = broadcast;

	const int cfgStatus = configureTCPAdapter(*netAdapter, netAdapter->id);
	if (cfgStatus != 0) {
		std::string msg("Failed to configure adapter " + adapter + " for parent " + parent + "\n");
		(void)msg;
	}

	return netAdapter;
}

int CommsAdapter::configureUDPAdapter(NetworkAdapter& netAdapter, int adapterIdx) {
	(void)netAdapter;
	(void)adapterIdx;
	return 0;
}

int CommsAdapter::configureTCPAdapter(NetworkAdapter& netAdapter, int adapterIdx) {
	(void)netAdapter;
	(void)adapterIdx;
	return 0;
}

CommandAdapter::CommandAdapter(std::string parentName_) : AdapterBase(ModuleDefs::AdapterId::CommandAdapterID, parentName_) {
}

int CommandAdapter::bind_(AdapterBase* Adapter) {
	bindInterface(static_cast<CommandAdapter*>(Adapter));
	return 0;
}

void CommandAdapter::bindInterface(CommandAdapter* adapter) {
	(void)adapter;
}

TlmAdapter::TlmAdapter(std::string parentName_) : AdapterBase(ModuleDefs::AdapterId::TlmAdapterID, parentName_) {
}

int TlmAdapter::registerTelemetrySource(const std::string& sourceName) {
	if (!registerSourceCommand) {
		return -1;
	}

	return registerSourceCommand(sourceName);
}

int TlmAdapter::publishTelemetry(const std::string& sourceName, const uint8_t* data, size_t length) {
	if (!publishTelemetryCommand || !data || length == 0) {
		return -1;
	}

	return publishTelemetryCommand(sourceName, data, length);
}

int TlmAdapter::publishTelemetry(const std::string& sourceName, const std::string& payload) {
	return publishTelemetry(sourceName, reinterpret_cast<const uint8_t*>(payload.data()), payload.size());
}

int TlmAdapter::bind_(AdapterBase* Adapter) {
	bindInterface(static_cast<TlmAdapter*>(Adapter));
	return 0;
}

void TlmAdapter::bindInterface(TlmAdapter* adapter) {
	if (!adapter) {
		return;
	}

	registerSourceCommand = [adapter](const std::string& sourceName) -> int {
		return adapter->registerTelemetrySource_(sourceName);
	};

	publishTelemetryCommand = [adapter](const std::string& sourceName, const uint8_t* data, size_t length) -> int {
		return adapter->publishTelemetry_(sourceName, data, length);
	};

	this->moduleWriteCmd = [adapter](char* pbuf, size_t len) {
		return adapter->moduleCommand_(pbuf, len);
	};

	this->moduleWriteAsyncCmd = [adapter](char* pbuf, size_t len) {
		return adapter->moduleCommandAsync_(pbuf, len);
	};
}

int TlmAdapter::registerTelemetrySource_(const std::string& sourceName) {
	(void)sourceName;
	return 0;
}

int TlmAdapter::publishTelemetry_(const std::string& sourceName, const uint8_t* data, size_t length) {
	(void)sourceName;
	(void)data;
	(void)length;
	return -1;
}

} // namespace Adapter
