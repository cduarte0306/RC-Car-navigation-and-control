# src/CMakeLists.txt
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "-g")

set(SOURCE
    main.cpp
    rc-car.cpp
    peripheral_driver.cpp
    network_interface/udp_socket.cpp
    cli/cli.cpp
)

add_executable(${PROJECT_NAME} ${SOURCE})

# Add your external includes (JSON) + embedded-cli header path
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/external/json/include
    ${CMAKE_SOURCE_DIR}/external/embedded-cli/lib/include
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        ${Boost_LIBRARIES}
        embedded_cli_lib   # <- correct name from the submodule
)
