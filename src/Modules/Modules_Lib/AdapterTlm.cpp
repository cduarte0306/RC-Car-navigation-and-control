#include "AdapterBase.hpp"

namespace Adapter {

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