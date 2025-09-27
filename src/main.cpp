#include <iostream>
#include <chrono>
#include <thread>
#include <type_traits>

#include "rc-car.hpp"
#include "cli/cli.hpp"

 int main(int argc, char* argv[]) {
    RcCar rcCar;
    AppCLI cli(rcCar);

    rcCar.joinThread();
    return 0;
}