#include "RcMotorController.hpp"

#include <iostream>
#include <chrono>
#include <string>

namespace Device {
MotorController::MotorController(int moduleID_, std::string name) : Base(moduleID, name), Adapter::MotorAdapter(name) {
    
}

MotorController::~MotorController() {

}
}
