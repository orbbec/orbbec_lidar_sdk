#include "driver.hpp"

#include <nlohmann/json.hpp>
#include <sstream>
#include <toml++/toml.hpp>

namespace ob_lidar_driver::detail {

std::shared_ptr<Logger> DriverImpl::logger_ = nullptr;

DriverImpl::DriverImpl(const std::string &config_file) {
    device_manager_ = std::make_unique<DeviceManager>();
    auto log_config = std::make_shared<LoggerConfig>();
    try {
        // Load the TOML configuration file
        auto config = toml::parse_file(config_file);
        if (auto logging = config["logging"].as_table()) {
            std::stringstream ss;
            ss << toml::json_formatter(*logging);
            auto logging_config_str = ss.str();

            log_config->loadFromJsonString(logging_config_str);
        } else {
            log_config = std::make_shared<LoggerConfig>();
        }
        if (!logger_) {
            logger_ = std::make_shared<Logger>(log_config);
        }
        // Access devices
        if (auto device_config_table = config["device"].as_table()) {
            for (const auto &[_, device_config_node] : *device_config_table) {
                std::stringstream ss;
                ss << toml::json_formatter(device_config_node);
                const auto json_config_str = ss.str();
                try {
                    auto json = nlohmann::json::parse(json_config_str);
                    auto device_name = json["name"].get<std::string>();
                    auto model_name = json["model"].get<std::string>();
                    auto network_config = json["network"];
                    DeviceConfigBuilder builder;
                    auto device_config =
                        builder.setDeviceName(device_name)
                            .setModel(model_name)
                            .setNetworkConfigJsonString(network_config.dump(2))
                            .build();
                    addDevice(device_config);

                } catch (const nlohmann::json::parse_error &err) {
                    LOG_ERROR("Failed to parse device config: {}", err.what());
                }
            }
        }

    } catch (const toml::parse_error &err) {
        LOG_ERROR("Failed to parse config file: {}", err.what());
    }
    LOG_INFO("DriverImpl initialized by config file: {}", config_file);
}

DriverImpl::DriverImpl() {
    auto log_config = std::make_shared<LoggerConfig>();
    device_manager_ = std::make_unique<DeviceManager>();
    if (!logger_) {
        logger_ = std::make_shared<Logger>(log_config);
    }
    LOG_INFO("DriverImpl initialized by default constructor");
}

DriverImpl::~DriverImpl() { stop(); }

void DriverImpl::enableDiscovery(bool enable) { discovery_enabled_ = enable; }

bool DriverImpl::isDiscoveryEnabled() const { return discovery_enabled_; }

void DriverImpl::discovery() {
    if (!discovery_enabled_) {
        return;
    }
    while (!stop_discovery_) {
        // TODO: implement discovery
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void DriverImpl::setOnDeviceChangedCallback(
    const onDeviceConnectedCallback &on_device_connected,
    const onDeviceDisconnectedCallback &on_device_disconnected) {
    on_device_connected_ = on_device_connected;
    on_device_disconnected_ = on_device_disconnected;
}

std::vector<std::shared_ptr<Device>> DriverImpl::getDevices() {
    const auto &devices = device_manager_->getDevices();
    std::vector<std::shared_ptr<Device>> device_ptrs;
    for (const auto &device : devices) {
        device_ptrs.push_back(device.second);
    }
    return device_ptrs;
}

std::shared_ptr<Device> DriverImpl::getDevice(const std::string &device_name) {
    const auto &devices = device_manager_->getDevices();
    const auto &device = devices.find(device_name);
    if (device != devices.end()) {
        return device->second;
    }
    return nullptr;
}

void DriverImpl::addDevice(std::shared_ptr<DeviceConfig> config) {
    device_manager_->addDevice(config);
}

void DriverImpl::removeDevice(const std::string &device_name) {
    device_manager_->removeDevice(device_name);
}

void DriverImpl::start() { device_manager_->start(); }

void DriverImpl::stop() { device_manager_->stop(); }

}  // namespace ob_lidar_driver::detail
