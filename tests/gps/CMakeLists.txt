cmake_minimum_required(VERSION 3.10)
project(rc-car-tests)

set(TEST_APP_NAME gps_test)

add_executable(${TEST_APP_NAME} main.cpp)

# Add debug flags or any compile options
target_compile_options(${TEST_APP_NAME} PRIVATE -m64)

# Force x86_64 build (if using GCC/Clang)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64")
endif()

# Link pthread
target_link_libraries(${TEST_APP_NAME} PRIVATE pthread)

# Place executable in build/tests/gps
set_target_properties(${TEST_APP_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests/gps)
