cmake_minimum_required(VERSION 3.15)


# Set the minimum required version of CMake for a project
find_package(PCL REQUIRED COMPONENTS common io visualization)

add_executable(post_process post_process.cpp)

target_include_directories(post_process PRIVATE ${CMAKE_SOURCE_DIR}/include
        ${PCL_INCLUDE_DIRS})
target_link_libraries(post_process PRIVATE orbbec_lidar_sdk
        ${PCL_LIBRARIES})
install(TARGETS post_process DESTINATION bin)
