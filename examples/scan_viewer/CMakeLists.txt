cmake_minimum_required(VERSION 3.15)


find_package(PCL REQUIRED COMPONENTS common io visualization)
add_executable(scan_viewer scan_viewer.cpp)
target_include_directories(scan_viewer PRIVATE ${CMAKE_SOURCE_DIR}/include
        ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
target_link_libraries(scan_viewer PRIVATE orbbec_lidar_sdk
        ${PCL_LIBRARIES})
install(TARGETS scan_viewer DESTINATION bin)
