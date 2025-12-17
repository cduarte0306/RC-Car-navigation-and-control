set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

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
    Modules/RcCommandAndControl.hpp
    Modules/RcCommandAndControl.cpp
    Modules/RcCommsController.hpp
    Modules/RcCommsController.cpp
    Modules/RcVisionControl.cpp
    Modules/RcVisionControl.hpp
    lib/VideoStreamer.cpp
    lib/VideoStreamer.hpp
    Devices/peripheralDriver.hpp
    Devices/peripheralDriver.cpp
    Devices/Pwm.cpp
    Devices/Pwm.hpp
    Devices/network_interface/UdpServer.cpp
    Devices/network_interface/UdpServer.hpp
    Devices/RegisterMap.hpp
    Devices/RegisterMap.cpp
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0)
pkg_check_modules(GSTAPP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GSTVIDEO REQUIRED gstreamer-video-1.0)

add_executable(${PROJECT_NAME} ${SOURCE})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/external/json/include
    ${CMAKE_SOURCE_DIR}/external/embedded-cli/lib/include
    ${CMAKE_SOURCE_DIR}/src/Devices/network_interface
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/Modules
    ${OPENCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        ${Boost_LIBRARIES}
        embedded_cli_lib
        systemd
        ${OPENCV_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${GST_INCLUDE_DIRS}
    ${GSTAPP_INCLUDE_DIRS}
    ${GSTVIDEO_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    ${GST_LIBRARIES}
    ${GSTAPP_LIBRARIES}
    ${GSTVIDEO_LIBRARIES}
)
