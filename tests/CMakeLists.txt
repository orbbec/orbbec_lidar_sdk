
include(CTest)
add_library(tcp_server ./tcp_server.cpp)

target_include_directories(tcp_server
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>

)

target_link_libraries(tcp_server
        PRIVATE
        uvw::uvw-static
        OrbLidarSDK
)


add_library(udp_server ./udp_server.cpp)

target_include_directories(
        udp_server
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(
        udp_server
        PRIVATE
        uvw::uvw-static
        OrbLidarSDK
)

add_executable(fake_server ./fake_server.cpp)

target_include_directories(
        fake_server
        PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(
        fake_server
        PRIVATE
        tcp_server
        udp_server
        uvw::uvw-static
        OrbLidarSDK
)

find_package(GTest REQUIRED)
function(ob_add_test TEST_NAME TEST_SOURCE)
    add_executable(${TEST_NAME} ${TEST_SOURCE})
    target_include_directories(${TEST_NAME} PRIVATE
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/contrib>
            $<INSTALL_INTERFACE:include>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/detail>
            ${THIRD_PARTY_DIR}/FastCRC/src
            ${MCAP_INCLUDE_DIR}
    )
    target_link_libraries(${TEST_NAME} PRIVATE
            tcp_server
            udp_server
            uvw::uvw-static
            OrbLidarSDK
            GTest::Main
            Threads::Threads
    )
    add_test(NAME ${TEST_NAME} COMMAND $<TARGET_FILE:${TEST_NAME}>
    )
endfunction()

ob_add_test(test_config test_config.cpp)

ob_add_test(test_device test_device.cpp)

ob_add_test(test_driver test_driver.cpp)

ob_add_test(test_filter test_filter.cpp)

ob_add_test(test_frame test_frame.cpp)

ob_add_test(test_option test_option.cpp)

ob_add_test(test_recorder test_recorder.cpp)
