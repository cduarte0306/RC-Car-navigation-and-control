#ifndef CLI_HPP
#define CLI_HPP

#include "../rc-car.hpp"
#include "embedded_cli.h"

class AppCLI {
public:
    AppCLI(RcCar& mainObj);
    ~AppCLI();
private:
    const char* tty = "/dev/ttyTHS1";
    RcCar& mainObj;
    EmbeddedCli *CLI = nullptr;
    int fd = -1;

    int openInterface();
    int writeIface(const char* format, ...);
};

#endif