#include <iostream>
#include <chrono>
#include <thread>
#include <type_traits>

#include "rc-car.hpp"


int main(int argc, char* argv[]) {
    RcCar rcCar;
    rcCar.joinThread();
    return 0;
}