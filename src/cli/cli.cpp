#include <iostream>
#include <sstream>
#include "cli.hpp"

#include "../navigation.hpp"

#define HELP_CMD        "help"
#define PRINT_WAYPOINTS "get-waypoints"


CLI::CLI(RcCar& rcCar):rcCar(rcCar) {
    this->cliThread = std::thread(&CLI::cliProcess, this);
}


CLI::~CLI() {
    this->threadCanRun = false;    
    this->cliThread.join();
}


void CLI::cliProcess(void) {
    std::string inputString;
    std::stringstream streamOut;
    std::cout << "Welcome to the RC Car CLI. Enter \"Help\" to display all available commands" << std::endl;

    while(this->threadCanRun) {
        std::cin >> inputString;
        
        if(inputString.find(HELP_CMD) != std::string::npos) {
            streamOut << "Available commands: " << std::endl;
        } else if(inputString.find(PRINT_WAYPOINTS) != std::string::npos) {
            GPS::Navigation* nav = this->rcCar.getNavObject();
            if(!nav) {
                std::cerr << "ERROR: Invalid navigation object\r\n";
            }

            
        } else {
            streamOut << "Invalid input" << std::endl;
        }

        std::cout << streamOut.str() << std::endl;
        streamOut.clear();
    }
}