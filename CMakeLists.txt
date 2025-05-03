cmake_minimum_required(VERSION 3.10)

set(PROJECT_NAME "rc-car-nav")

# Set the project name
project(${PROJECT_NAME})

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add the subdirectory containing the bottom-level CMakeLists.txt
add_subdirectory(src)