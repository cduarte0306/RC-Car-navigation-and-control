set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "-g")

set(SOURCE
    main.cpp
    utils/logger.cpp
    Modules/cli.hpp
    Modules/cli.cpp
    Modules/RcBase.cpp
    Modules/RcBase.hpp
    Modules/RcMotorController.cpp
    Modules/RcMotorController.hpp
    Modules/RcMessageLib.hpp
    Modules/RcWirelessComms.hpp
    Modules/RcWirelessComms.cpp
    Devices/peripheralDriver.hpp
    Devices/peripheralDriver.cpp
    Devices/network_interface/UdpServer.cpp
    Devices/network_interface/UdpServer.hpp
)

add_executable(${PROJECT_NAME} ${SOURCE})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/external/json/include
    ${CMAKE_SOURCE_DIR}/external/embedded-cli/lib/include
    ${CMAKE_SOURCE_DIR}/src/Devices/network_interface
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/Modules
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        ${Boost_LIBRARIES}
        embedded_cli_lib
        systemd
)
