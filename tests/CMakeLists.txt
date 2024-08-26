
include(CTest)

include_directories(
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/contrib>
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/detail>
        ${THIRD_PARTY_DIR}/FastCRC/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include>
        ${MCAP_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/src/detail
        ${CMAKE_SOURCE_DIR}/src/contrib
)

add_library(tcp_server ./tcp_server.cpp)

target_link_libraries(tcp_server
        PRIVATE
        uvw::uvw-static
        orbbec_lidar_sdk
)


add_library(udp_server ./udp_server.cpp)


target_link_libraries(
        udp_server
        PRIVATE
        uvw::uvw-static
        orbbec_lidar_sdk
        orbbec_lidar_sdk
)

add_executable(fake_server ./fake_server.cpp)


target_link_libraries(
        fake_server
        PRIVATE
        tcp_server
        udp_server
        uvw::uvw-static
        orbbec_lidar_sdk
)

find_package(GTest REQUIRED)
function(ob_add_test TEST_NAME TEST_SOURCE)
    add_executable(${TEST_NAME} ${TEST_SOURCE})
    target_link_libraries(${TEST_NAME} PRIVATE
            tcp_server
            udp_server
            uvw::uvw-static
            orbbec_lidar_sdk
            GTest::Main
            Threads::Threads
    )
    add_test(NAME ${TEST_NAME} COMMAND $<TARGET_FILE:${TEST_NAME}>
    )
endfunction()

ob_add_test(test_config test_config.cpp)

ob_add_test(test_device test_device.cpp)

ob_add_test(test_device_manager test_device_manager.cpp)

ob_add_test(test_filter test_filter.cpp)

ob_add_test(test_frame test_frame.cpp)

ob_add_test(test_option test_option.cpp)

ob_add_test(test_recorder test_recorder.cpp)
