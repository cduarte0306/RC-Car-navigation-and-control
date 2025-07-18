#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "cli.hpp"

#include "../navigation.hpp"

#define HELP_CMD        "help"
#define PRINT_WAYPOINTS "get-waypoints"
#define SET_WAYPOINT    "set-waypoint"


CLI::CLI(RcCar& rcCar):
 rcCar(rcCar) {
    this->cliThread = std::thread(&CLI::cliProcess, this);
}


CLI::~CLI() {
    this->threadCanRun = false;    
    this->cliThread.join();
}


void CLI::cliProcess(void) {
    std::string inputString;
    std::string token;
    std::stringstream streamOut;
    std::vector<std::string> argBank;
    std::cout << "Welcome to the RC Car CLI. Enter \"Help\" to display all available commands" << std::endl;

    while(this->threadCanRun) {
        // Parse the inputs
        std::getline(std::cin, inputString);
        std::istringstream iss(inputString);

        auto pos = inputString.find("'");
        if (pos != std::string::npos) {
            auto closingPos = inputString.substr(pos + 1, inputString.length()).find("'");
            if (closingPos != std::string::npos) {
                argBank.push_back(inputString.substr(pos + 1, closingPos));
            }
        } else {
            while (std::getline(iss, token, ' ')) {
                argBank.push_back(token);
            }
        }
        
        if(inputString.find(HELP_CMD) != std::string::npos) {
            streamOut << "Available commands: " << std::endl;
        } else if(inputString.find(PRINT_WAYPOINTS) != std::string::npos) {
            GPS::Navigation* nav = this->rcCar.getNavObject();
            if(nav) {
                std::vector<std::string>ways = nav->ways();
                for( const auto& way : ways) {
                    streamOut << way << "\r\n";
                }
            } else {
                streamOut << "ERROR: Invalid navigation object";
            }            
        } else if(inputString.find(SET_WAYPOINT) != std::string::npos) {
            GPS::Navigation* nav = this->rcCar.getNavObject();

            if(nav) {
                int ret = nav->calculatePath(argBank[0]);
                if (ret < 0) {
                    streamOut << "ERROR: Invalid waypoint";
                }
            } else {
                streamOut << "ERROR: Invalid navigation object";
            } 
        } else {
            streamOut << "Invalid input" << std::endl;
        }

        std::cout << streamOut.str() << std::endl;
        
        streamOut.clear();
        argBank.clear();
        token.clear();
    }
}