#ifndef CLI_HPP
#define CLI_HPP

#include "../rc-car.hpp"
#include "embedded_cli.h"

class AppCLI {
public:
    AppCLI(RcCar& mainObj);
    ~AppCLI();
private:
    RcCar& mainObj;
};

#endif