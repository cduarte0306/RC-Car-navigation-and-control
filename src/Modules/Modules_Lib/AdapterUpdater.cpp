#include "AdapterBase.hpp"

namespace Adapter {

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

} // namespace Adapter