# Add debug flags
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "-g")

# Source files
set(SOURCE
    main.cpp
    rc-car.cpp
    rc-car.hpp
    peripheral_driver.cpp
    peripheral_driver.hpp
    network_interface/udp_socket.hpp
    network_interface/udp_socket.cpp
    gps_interface/gps_interface.cpp
    navigation.cpp
)

# Only look for Boost if not cross-compiling
if(NOT CMAKE_CROSSCOMPILING)
    find_package(Boost REQUIRED COMPONENTS system filesystem thread)
    include_directories(${Boost_INCLUDE_DIRS})
endif()

include_directories(
    ${CMAKE_SOURCE_DIR}/external/libosmium/include
)

# Define the executable
add_executable(${PROJECT_NAME} ${SOURCE})

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

# JSON dependency
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/external/json/include
)
