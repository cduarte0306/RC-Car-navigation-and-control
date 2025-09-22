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
            cli/cli.cpp
            )

add_executable(${PROJECT_NAME} ${SOURCE})

# Add this near the top
target_include_directories(rc-car-nav PRIVATE
    ${CMAKE_SOURCE_DIR}/external/json/include
    ${CMAKE_SOURCE_DIR}/external/json/include/lib
    ${CMAKE_SOURCE_DIR}/external/embedded-cli/lib/include
)