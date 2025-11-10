set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "-g")

set(SOURCE
    main.cpp
    rc-car.cpp
    peripheral_driver.cpp
    network_interface/udp_socket.cpp
    cli/cli.cpp
    utils/logger.cpp
    rc-car-lib/RcBase.cpp
    rc-car-lib/RcBase.hpp
    rc-car-lib/RcMotorController.cpp
    rc-car-lib/RcMotorController.hpp
    rc-car-lib/RcMessageLib.hpp
    rc-car-lib/RcWirelessComms.hpp
)

add_executable(${PROJECT_NAME} ${SOURCE})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/external/json/include
    ${CMAKE_SOURCE_DIR}/external/embedded-cli/lib/include
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/rc-car-lib
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        ${Boost_LIBRARIES}
        embedded_cli_lib
        systemd
)
