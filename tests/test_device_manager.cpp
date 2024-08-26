#include <gtest/gtest.h>

#include <filesystem>
#include <future>
#include <nlohmann/json.hpp>
#include <toml++/toml.hpp>

#include "detail/device_manager.hpp"
#include "orbbec_lidar/device_manager.hpp"
#include "tcp_server.hpp"
#include "test_utils.hpp"
namespace ob = ob_lidar;
using namespace ob_test;

TEST(DeviceManager, test_default) {
    auto device_manager = std::make_shared<ob::DeviceManager>();
    EXPECT_NE(device_manager, nullptr);
    EXPECT_EQ(device_manager->getDevices().size(), 0);
}

TEST_F(FakeTCPSever, test_driver_read_config_1) {
    char buf[PATH_MAX];
    char *ptr = realpath(__FILE__, buf);
    std::string cwd = std::filesystem::path(ptr).parent_path();
    const std::string toml_file =
        cwd + "/config/test_single_device_tcp_config.toml";
    auto device_manager = std::make_shared<ob::DeviceManager>(toml_file);
    EXPECT_NE(device_manager, nullptr);
    EXPECT_EQ(device_manager->getDevices().size(), 1);
    auto device = device_manager->getDevice("ob_lidar");
    EXPECT_NE(device, nullptr);
    auto info = device->getInfo();
    EXPECT_EQ(info.serial_number, "1234567890abcdef");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

TEST_F(FakeTCPSever, test_read_config_2) {
    char buf[PATH_MAX];
    char *ptr = realpath(__FILE__, buf);
    std::string cur_path = std::filesystem::path(ptr).parent_path();
    const std::string toml_file =
        cur_path + "/config/test_multi_device_tcp_config.toml";
    auto device_manager = std::make_shared<ob::DeviceManager>(toml_file);
    EXPECT_NE(device_manager, nullptr);
    EXPECT_EQ(device_manager->getDevices().size(), 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

TEST_F(FakeUDPServer, test_read_config_3) {
    char buf[PATH_MAX];
    char *ptr = realpath(__FILE__, buf);
    std::string cur_path = std::filesystem::path(ptr).parent_path();
    const std::string toml_file =
        cur_path + "/config/test_single_device_udp_config.toml";
    auto device_manager = std::make_shared<ob::DeviceManager>(toml_file);
    EXPECT_NE(device_manager, nullptr);
    EXPECT_EQ(device_manager->getDevices().size(), 1);
    auto device = device_manager->getDevice("ob_lidar");
    EXPECT_NE(device, nullptr);
    auto info = device->getInfo();
    EXPECT_EQ(info.serial_number, "1234567890abcdfg");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

TEST_F(FakeUDPServer, test_read_config_4) {
    char buf[PATH_MAX];
    char *ptr = realpath(__FILE__, buf);
    std::string cur_path = std::filesystem::path(ptr).parent_path();
    const std::string toml_file =
        cur_path + "/config/test_multi_device_udp_config.toml";
    auto device_manager = std::make_shared<ob::DeviceManager>(toml_file);
    EXPECT_NE(device_manager, nullptr);
    EXPECT_EQ(device_manager->getDevices().size(), 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}
