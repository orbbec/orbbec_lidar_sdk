#include <gtest/gtest.h>

#include <filesystem>
#include <future>
#include <nlohmann/json.hpp>
#include <toml++/toml.hpp>

#include "detail/driver.hpp"
#include "orb_lidar_driver/driver.hpp"
#include "tcp_server.hpp"
#include "test_utils.hpp"
namespace ob = ob_lidar_driver;
using namespace ob_test;

TEST(Driver, test_default) {
    auto driver = std::make_shared<ob::Driver>();
    EXPECT_NE(driver, nullptr);
    EXPECT_EQ(driver->getDevices().size(), 0);
}

TEST_F(FakeTCPSever, test_driver_read_config_1) {
    char buf[PATH_MAX];
    char *ptr = realpath(__FILE__, buf);
    std::string cwd = std::filesystem::path(ptr).parent_path();
    const std::string toml_file =
        cwd + "/config/test_single_device_tcp_config.toml";
    auto driver = std::make_shared<ob::Driver>(toml_file);
    EXPECT_NE(driver, nullptr);
    EXPECT_EQ(driver->getDevices().size(), 1);
    auto device = driver->getDevice("ob_lidar");
    EXPECT_NE(device, nullptr);
    auto info = device->getInfo();
    EXPECT_EQ(info.serial_number, "1234567890abcdef");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

TEST_F(FakeTCPSever, test_driver_read_config_2) {
    char buf[PATH_MAX];
    char *ptr = realpath(__FILE__, buf);
    std::string cur_path = std::filesystem::path(ptr).parent_path();
    const std::string toml_file =
        cur_path + "/config/test_multi_device_tcp_config.toml";
    auto driver = std::make_shared<ob::Driver>(toml_file);
    EXPECT_NE(driver, nullptr);
    EXPECT_EQ(driver->getDevices().size(), 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

TEST_F(FakeUDPServer, test_driver_read_config_3) {
    char buf[PATH_MAX];
    char *ptr = realpath(__FILE__, buf);
    std::string cur_path = std::filesystem::path(ptr).parent_path();
    const std::string toml_file =
        cur_path + "/config/test_single_device_udp_config.toml";
    auto driver = std::make_shared<ob::Driver>(toml_file);
    EXPECT_NE(driver, nullptr);
    EXPECT_EQ(driver->getDevices().size(), 1);
    auto device = driver->getDevice("ob_lidar");
    EXPECT_NE(device, nullptr);
    auto info = device->getInfo();
    EXPECT_EQ(info.serial_number, "1234567890abcdfg");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

TEST_F(FakeUDPServer, test_driver_read_config_4) {
    char buf[PATH_MAX];
    char *ptr = realpath(__FILE__, buf);
    std::string cur_path = std::filesystem::path(ptr).parent_path();
    const std::string toml_file =
        cur_path + "/config/test_multi_device_udp_config.toml";
    auto driver = std::make_shared<ob::Driver>(toml_file);
    EXPECT_NE(driver, nullptr);
    EXPECT_EQ(driver->getDevices().size(), 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}
