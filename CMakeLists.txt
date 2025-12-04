cmake_minimum_required(VERSION 3.10)

project(rc-car-nav LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Boost (unchanged)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)

# Use pkg-config instead of broken OpenCVConfig.cmake
find_package(PkgConfig REQUIRED)
pkg_check_modules(OPENCV REQUIRED opencv4)

include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})
add_definitions(${OPENCV_CFLAGS_OTHER})

add_subdirectory(external/embedded-cli)
add_subdirectory(src)
