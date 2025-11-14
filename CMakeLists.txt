cmake_minimum_required(VERSION 3.10)

# Make sure we enable both C and C++
project(rc-car-nav LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Boost for your app (used in src/)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)

# Add subdirectories
add_subdirectory(external/embedded-cli)   # builds the static lib: embedded_cli_lib
add_subdirectory(src)                     # builds your app: rc-car-nav