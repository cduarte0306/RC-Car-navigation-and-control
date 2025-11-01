#include <iostream>
#include <chrono>
#include <thread>
#include <type_traits>

#include "rc-car.hpp"
#include "cli/cli.hpp"

#include "utils/logger.hpp"
#include "version.h"


int main(int argc, char* argv[]) {
    Logger* logger = Logger::getLoggerInst();
    logger->log(Logger::LOG_LVL_INFO, "RC Car navigation and control V%u.%u.%u\r\n", VERSION_MAJOR, VERSION_MINOR, VERSION_BUILD);

    RcCar rcCar;
    AppCLI cli(rcCar);

    rcCar.joinThread();
    return 0;
}