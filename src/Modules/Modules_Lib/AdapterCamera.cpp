#include "AdapterBase.hpp"

namespace Adapter {

CameraAdapter::CameraAdapter(std::string parentName_) : AdapterBase(ModuleDefs::AdapterId::CameraAdapterID, parentName_) {
}

int CameraAdapter::setCameraState(bool state) {
	(void)state;
	return 0;
}

int CameraAdapter::StartStreaming() {
	if (startStreamingCommand) {
		return startStreamingCommand();
	}
	return 0;
}

int CameraAdapter::StopStreaming() {
	if (stopStreamingCommand) {
		return stopStreamingCommand();
	}
	return 0;
}

int CameraAdapter::SetStreamingState(bool state) {
	return state ? StartStreaming() : StopStreaming();
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

	this->startStreamingCommand = [adapter]() -> int {
		return adapter->startStreaming_();
	};

	this->stopStreamingCommand = [adapter]() -> int {
		return adapter->stopStreaming_();
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

int CameraAdapter::startStreaming_() {
	return 0;
}

int CameraAdapter::stopStreaming_() {
	return 0;
}

} // namespace Adapter