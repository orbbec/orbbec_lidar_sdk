#include "device_manager.hpp"

#include <nlohmann/json.hpp>
#include <sstream>
#include <toml++/toml.hpp>

#include "device.hpp"

namespace ob_lidar::detail {

std::shared_ptr<Logger> DeviceManagerImpl::logger_ = nullptr;

DeviceManagerImpl::DeviceManagerImpl(const std::string &config_file) {
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

DeviceManagerImpl::DeviceManagerImpl() {
    auto log_config = std::make_shared<LoggerConfig>();
    if (!logger_) {
        logger_ = std::make_shared<Logger>(log_config);
    }
    LOG_INFO("DriverImpl initialized by default constructor");
}

DeviceManagerImpl::~DeviceManagerImpl() { stop(); }

void DeviceManagerImpl::enableDiscovery(bool enable) { discovery_enabled_ = enable; }

bool DeviceManagerImpl::isDiscoveryEnabled() const { return discovery_enabled_; }

void DeviceManagerImpl::discovery() {
    if (!discovery_enabled_) {
        return;
    }
    while (!stop_discovery_) {
        // TODO: implement discovery
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void DeviceManagerImpl::setOnDeviceChangedCallback(
    const onDeviceConnectedCallback &on_device_connected,
    const onDeviceDisconnectedCallback &on_device_disconnected) {
    on_device_connected_ = on_device_connected;
    on_device_disconnected_ = on_device_disconnected;
}

std::vector<std::shared_ptr<Device>> DeviceManagerImpl::getDevices() {
    std::vector<std::shared_ptr<Device>> device_ptrs;
    for (const auto &device : devices_) {
        device_ptrs.push_back(device.second);
    }
    return device_ptrs;
}

std::shared_ptr<Device> DeviceManagerImpl::getDevice(const std::string &device_name) {
    const auto &device = devices_.find(device_name);
    if (device != devices_.end()) {
        return device->second;
    }
    return nullptr;
}

void DeviceManagerImpl::addDevice(std::shared_ptr<DeviceConfig> config) {
    auto device_impl = std::make_unique<DeviceImpl>(config);
    auto device = std::make_shared<Device>(std::move(device_impl));
    if (devices_.find(device->getName()) != devices_.end()) {
        return;
    }
    LOG_INFO("add device {}", device->getName());
    devices_[device->getName()] = device;
}

void DeviceManagerImpl::removeDevice(const std::string &device_name) {
    if (devices_.find(device_name) == devices_.end()) {
        LOG_ERROR("device {} not found", device_name);
        return;
    }
    devices_.erase(device_name);
}

void DeviceManagerImpl::start() {
    LOG_INFO("start all devices");
    for (auto &[device_name, device] : devices_) {
        const auto status = device->start();
        if (status != Status::OK) {
            LOG_ERROR("failed to start device {}", device_name);
        }
    }
}

void DeviceManagerImpl::stop() {
    LOG_INFO("stop all devices");
    for (auto &[device_name, device] : devices_) {
        const auto status = device->stop();
        if (status != Status::OK) {
            LOG_ERROR("failed to stop device {}", device_name);
        }
    }
    LOG_INFO("stop all devices done");
}

}  // namespace ob_lidar::detail
