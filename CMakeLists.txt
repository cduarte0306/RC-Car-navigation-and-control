cmake_minimum_required(VERSION 3.10)

project(rc-car-nav LANGUAGES C CXX CUDA)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# Boost (unchanged)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)

# Use pkg-config instead of broken OpenCVConfig.cmake
find_package(PkgConfig REQUIRED)

set(VPI_ROOT "${CMAKE_SYSROOT}/opt/nvidia/vpi3")
set(vpi_DIR "${VPI_ROOT}/lib/aarch64-linux-gnu/cmake/vpi")

pkg_check_modules(OPENCV REQUIRED opencv4)
find_package(vpi CONFIG REQUIRED)

include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})
add_definitions(${OPENCV_CFLAGS_OTHER})

add_subdirectory(external/embedded-cli)
add_subdirectory(src)

