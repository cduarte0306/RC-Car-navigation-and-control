cmake_minimum_required(VERSION 3.10)

# Set the project name
project(rc-car-nav)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find Boost libraries (replace placeholders with actual components)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)

# Add Boost headers globally
include_directories(${Boost_INCLUDE_DIRS})

include_directories(${CMAKE_SOURCE_DIR})

# When linking your target:
# target_link_libraries(rc-car-nav PRIVATE nlohmann_json::nlohmann_json)

# Add the subdirectory containing the bottom-level CMakeLists.txt
add_subdirectory(src)
