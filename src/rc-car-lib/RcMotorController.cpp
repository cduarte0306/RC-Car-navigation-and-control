#include "RcMotorController.hpp"

#include <iostream>
#include <chrono>
#include <string>

namespace Device {
MotorController::MotorController(int moduleID_, std::string name) : Base(moduleID, name), Adapter::MotorAdapter(name) {
    
}

MotorController::~MotorController() {

}

void MotorController::mainProc() {
    // Main processing loop for the motor controller
    while (true) {
        // Process motor commands
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
}
