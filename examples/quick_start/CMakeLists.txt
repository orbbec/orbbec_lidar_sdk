cmake_minimum_required(VERSION 3.15)


# Set the minimum required version of CMake for a project

add_executable(quick_start quick_start.cpp)

target_include_directories(quick_start PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(quick_start PRIVATE OrbLidarSDK)
install(TARGETS quick_start DESTINATION bin)
