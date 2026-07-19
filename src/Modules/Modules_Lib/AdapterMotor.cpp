#include "AdapterBase.hpp"

namespace Adapter {

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

} // namespace Adapter