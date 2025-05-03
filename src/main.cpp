#include <iostream>
#include <chrono>
#include <thread>

#include "rc-car.hpp"


int main(int argc, char* argv[]) {
    RcCar rcCar;

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    return 0;
}