#ifndef CLI_HPP
#define CLI_HPP

#include <thread>
#include <string>
#include <set>

#include "../rc-car.hpp"


class CLI {
public:
    CLI(RcCar& rcCar);
    ~CLI();
private:
    bool threadCanRun = true;
    std::thread cliThread;

    RcCar& rcCar;
    void cliProcess(void);
};

#endif