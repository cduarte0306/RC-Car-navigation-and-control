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
    Modules/RcCarTelemetry.cpp
    Modules/RcCarTelemetry.hpp
    app/VideoFrame.cpp
    app/VideoFrame.hpp
    app/VideoStreamer.cpp
    app/VideoStreamer.hpp
    app/VideoRecording.cpp
    app/VideoRecording.hpp
    Devices/peripheralDriver.hpp
    Devices/peripheralDriver.cpp
    Devices/Pwm.cpp
    Devices/Pwm.hpp
    Devices/network_interface/UdpServer.cpp
    Devices/network_interface/UdpServer.hpp
    Devices/RegisterMap.hpp
    Devices/RegisterMap.cpp
    Devices/GyroScope.cpp
    Devices/GyroScope.hpp
    Devices/StereoCam.cpp
    Devices/StereoCam.hpp
    lib/Thread.cpp
    lib/Thread.hpp
    Devices/Gpio.cpp
    Devices/Gpio.hpp
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0)
pkg_check_modules(GSTAPP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GSTVIDEO REQUIRED gstreamer-video-1.0)
pkg_check_modules(GPIOD REQUIRED libgpiod)

find_library(GPIOD_LIBRARY gpiod)

add_executable(${PROJECT_NAME} ${SOURCE})

option(RC_CAR_ENABLE_JETSON_MULTIMEDIA "Enable Jetson multimedia (Argus/NvBuf) integration when libraries are available" ON)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/external/json/include
    ${CMAKE_SOURCE_DIR}/external/embedded-cli/lib/include
    ${CMAKE_SOURCE_DIR}/src/Devices/network_interface
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/lib
    ${CMAKE_SOURCE_DIR}/src/Modules
    ${OPENCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PRIVATE ${GPIOD_LIBRARY})

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        ${Boost_LIBRARIES}
        embedded_cli_lib
        systemd
        pthread
        EGL
        GLESv2
        ${OPENCV_LIBRARIES}
)

if (RC_CAR_ENABLE_JETSON_MULTIMEDIA)
    if (RC_CAR_ENABLE_JETSON_MULTIMEDIA)
        # Use SDK sysroot if present
        if (NOT CMAKE_SYSROOT AND DEFINED ENV{SDKTARGETSYSROOT})
            set(CMAKE_SYSROOT "$ENV{SDKTARGETSYSROOT}")
        endif()

        find_library(RC_CAR_NVARGUS_SOCKETCLIENT_LIBRARY
            NAMES nvargus_socketclient
            HINTS "${CMAKE_SYSROOT}/usr/lib"
        )

        find_library(RC_CAR_NVBUF_SURFACE_LIBRARY
            NAMES nvbufsurface
            HINTS "${CMAKE_SYSROOT}/usr/lib"
        )

        find_library(RC_CAR_NVBUF_SURFTRANSFORM_LIBRARY
            NAMES nvbufsurftransform
            HINTS "${CMAKE_SYSROOT}/usr/lib"
        )

        if (RC_CAR_NVARGUS_SOCKETCLIENT_LIBRARY AND RC_CAR_NVBUF_SURFACE_LIBRARY AND RC_CAR_NVBUF_SURFTRANSFORM_LIBRARY)
            target_compile_definitions(${PROJECT_NAME} PRIVATE RC_CAR_HAVE_JETSON_MULTIMEDIA=1)

            # If your code uses #include <Argus/Argus.h>, add ${CMAKE_SYSROOT}/usr/include
            # If it uses #include <Argus.h>, add ${CMAKE_SYSROOT}/usr/include/Argus
            target_include_directories(${PROJECT_NAME} PRIVATE
                "${CMAKE_SYSROOT}/usr/include"
                "${CMAKE_SYSROOT}/usr/include/Argus"
            )

            target_link_libraries(${PROJECT_NAME} PRIVATE
                ${RC_CAR_NVARGUS_SOCKETCLIENT_LIBRARY}
                ${RC_CAR_NVBUF_SURFACE_LIBRARY}
                ${RC_CAR_NVBUF_SURFTRANSFORM_LIBRARY}
                pthread
                dl
            )
        else()
            message(STATUS "Jetson multimedia libs not found (nvargus_socketclient/nvbufsurface/nvbufsurftransform); building without Jetson multimedia support")
            target_compile_definitions(${PROJECT_NAME} PRIVATE RC_CAR_HAVE_JETSON_MULTIMEDIA=0)
        endif()
    endif()

    if (RC_CAR_ARGUS_LIBRARY AND RC_CAR_NVBUF_UTILS_LIBRARY)
        target_compile_definitions(${PROJECT_NAME} PRIVATE RC_CAR_HAVE_JETSON_MULTIMEDIA=1)
        target_link_libraries(${PROJECT_NAME} PRIVATE ${RC_CAR_ARGUS_LIBRARY} ${RC_CAR_NVBUF_UTILS_LIBRARY})
    else()
        message(STATUS "Jetson multimedia libs not found (argus/nvbuf_utils); building without Jetson multimedia support")
        target_compile_definitions(${PROJECT_NAME} PRIVATE RC_CAR_HAVE_JETSON_MULTIMEDIA=0)
    endif()
else()
    target_compile_definitions(${PROJECT_NAME} PRIVATE RC_CAR_HAVE_JETSON_MULTIMEDIA=0)
endif()

target_include_directories(${PROJECT_NAME} PRIVATE
    ${GST_INCLUDE_DIRS}
    ${GSTAPP_INCLUDE_DIRS}
    ${GSTVIDEO_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    ${GST_LIBRARIES}
    ${GSTAPP_LIBRARIES}
    ${GSTVIDEO_LIBRARIES}
    ${GPIOD_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${GPIOD_INCLUDE_DIRS}
)

target_compile_options(${PROJECT_NAME} PRIVATE
    ${GPIOD_CFLAGS_OTHER}
)
