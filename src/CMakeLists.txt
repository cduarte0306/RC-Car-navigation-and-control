set(CMAKE_CXX_STANDARD 11)

# Add debug flags
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "-g")

set(SOURCE  main.cpp
            rc-car.cpp
            rc-car.hpp
            peripheral_driver.cpp
            peripheral_driver.hpp
            network_interface/udp_socket.hpp
            network_interface/udp_socket.cpp
            )

add_executable(${PROJECT_NAME} ${SOURCE})