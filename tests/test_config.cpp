#include <gtest/gtest.h>

#include <filesystem>
#include <nlohmann/json.hpp>
#include <toml++/toml.hpp>

#include "../third_party/spdlog/include/spdlog/fmt/bundled/chrono.h"
#include "orb_lidar_driver/driver.hpp"

TEST(loggerConfig, default_config) {
    using namespace ob_lidar_driver;
    auto config = std::make_shared<LoggerConfig>();
    EXPECT_EQ(config->getConsoleLogLevel(), LogLevel::INFO);
    EXPECT_EQ(config->getFileLogLevel(), LogLevel::INFO);
    // get current working directory
    const std::string cwd = std::filesystem::current_path();
    EXPECT_EQ(config->getLogFileDir(), cwd + "/logs");
    EXPECT_EQ(config->getMaxLogFileNum(), 10);
    EXPECT_EQ(config->getMaxLogFileSize(), 1024 * 1024 * 10);
    EXPECT_EQ(config->shouldLogToConsole(), true);
    EXPECT_EQ(config->shouldLogToFile(), true);
}

TEST(loggerConfig, load_from_json_string) {
    using namespace ob_lidar_driver;
    nlohmann::json j;
    j["console_log_level"] = "DEBUG";
    j["file_log_level"] = "ERROR";
    j["log_file_dir"] = "/tmp";
    j["max_file_num"] = 5;
    j["max_file_size"] = 1024 * 1024 * 5;  // 5MB
    j["log_to_console"] = false;
    j["log_to_file"] = false;
    auto config = std::make_shared<LoggerConfig>();
    config->loadFromJsonString(j.dump());
    EXPECT_EQ(config->getConsoleLogLevel(), LogLevel::DEBUG);
    EXPECT_EQ(config->getFileLogLevel(), LogLevel::ERROR);
    EXPECT_EQ(config->getLogFileDir(), "/tmp");
    EXPECT_EQ(config->getMaxLogFileNum(), 5);
    EXPECT_EQ(config->getMaxLogFileSize(), 1024 * 1024 * 5);
    EXPECT_EQ(config->shouldLogToConsole(), false);
    EXPECT_EQ(config->shouldLogToFile(), false);
}

TEST(loggerConfig, load_from_toml_file) {
    using namespace ob_lidar_driver;
    char buf[PATH_MAX];
    char *ptr = realpath(__FILE__, buf);
    std::string parent_path =
        std::filesystem::path(ptr).parent_path().parent_path();
    const std::string toml_file =
        parent_path + "/config/single_device_config.toml";
    auto config = std::make_shared<LoggerConfig>(toml_file);
    EXPECT_NE(config, nullptr);
    EXPECT_EQ(config->getConsoleLogLevel(), LogLevel::INFO);
    EXPECT_EQ(config->getFileLogLevel(), LogLevel::INFO);
    EXPECT_EQ(config->getLogFileDir(), "/tmp");
    EXPECT_EQ(config->getMaxLogFileNum(), 10);
    EXPECT_EQ(config->getMaxLogFileSize(), 1024 * 1024 * 10);
    EXPECT_EQ(config->shouldLogToConsole(), true);
    EXPECT_EQ(config->shouldLogToFile(), true);
}

TEST(NetworkConfig, load_from_json_string) {
    using namespace ob_lidar_driver;
    nlohmann::json j;
    j["single_port_mode"] = true;
    j["ip"] = "192.168.1.100";
    j["protocol"] = "udp";
    j["port"] = 2401;
    auto config = std::make_shared<NetworkConfig>();
    config->loadFromJsonString(j.dump());
    EXPECT_EQ(config->isSinglePort(), true);
    EXPECT_EQ(config->getIp(), "192.168.1.100");
    EXPECT_EQ(config->getProtocolType(), LidarProtocolType::UDP);
    EXPECT_EQ(config->getPort(), 2401);
}

TEST(DeviceConfig, load_from_toml_file) {
    using namespace ob_lidar_driver;
    char buf[PATH_MAX];
    char *ptr = realpath(__FILE__, buf);
    std::string parent_path =
        std::filesystem::path(ptr).parent_path().parent_path();
    const std::string toml_file =
        parent_path + "/config/single_device_config.toml";
    auto config = std::make_shared<DeviceConfig>(toml_file);
    EXPECT_NE(config, nullptr);
    EXPECT_EQ(config->getDeviceName(), "ob_lidar");
    EXPECT_EQ(config->getProtocolType(), LidarProtocolType::UDP);
    EXPECT_EQ(config->getModel(), LidarModel::MS600);
    auto network_config = config->getNetworkConfig();
    EXPECT_NE(network_config, nullptr);
    EXPECT_EQ(network_config->isSinglePort(), true);
    EXPECT_EQ(network_config->getIp(), "192.168.1.100");
    EXPECT_EQ(network_config->getProtocolType(), LidarProtocolType::UDP);
    EXPECT_EQ(network_config->getPort(), 2401);
}

TEST(DeviceConfig, test_builder_1) {
    using namespace ob_lidar_driver;
    DeviceConfigBuilder builder;

    nlohmann::json j;
    j["single_port_mode"] = true;
    j["ip"] = "192.168.1.100";
    j["protocol"] = "udp";
    j["port"] = 2401;
    auto config = builder.setDeviceName("my_lidar")
                      .setModel("MS600")
                      .setNetworkConfigJsonString(j.dump())
                      .build();
    EXPECT_NE(config, nullptr);
    EXPECT_EQ(config->getDeviceName(), "my_lidar");
    EXPECT_EQ(config->getProtocolType(), LidarProtocolType::UDP);
    EXPECT_EQ(config->getModel(), LidarModel::MS600);
    auto network_config = config->getNetworkConfig();
    EXPECT_NE(network_config, nullptr);
    EXPECT_EQ(network_config->isSinglePort(), true);
    EXPECT_EQ(network_config->getIp(), "192.168.1.100");
    EXPECT_EQ(network_config->getProtocolType(), LidarProtocolType::UDP);
    EXPECT_EQ(network_config->getPort(), 2401);
}

TEST(DeviceConfig, test_builder_2) {
    using namespace ob_lidar_driver;
    DeviceConfigBuilder builder;
    auto config = builder.setDeviceName("my_lidar")
                      .setModel("TL2401")
                      .setProtocolType("UDP")
                      .build();
    EXPECT_NE(config, nullptr);
    EXPECT_EQ(config->getDeviceName(), "my_lidar");
    EXPECT_EQ(config->getProtocolType(), LidarProtocolType::UDP);
    EXPECT_EQ(config->getModel(), LidarModel::TL2401);
    auto network_config = config->getNetworkConfig();
    EXPECT_NE(network_config, nullptr);
    EXPECT_EQ(network_config->isSinglePort(), true);
    EXPECT_EQ(network_config->getIp(), "192.168.1.100");
    EXPECT_EQ(network_config->getProtocolType(), LidarProtocolType::UDP);
    EXPECT_EQ(network_config->getPort(), 2401);
}
