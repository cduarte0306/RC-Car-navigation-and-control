#include "Modules_Lib/AdapterBase.hpp"

namespace Adapter {


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

} // namespace Adapter