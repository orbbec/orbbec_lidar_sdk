#include "device_manager.hpp"

#include "../logger.hpp"
#include "detail/device.hpp"
namespace ob_lidar_driver {
DeviceManager::DeviceManager() = default;

DeviceManager::~DeviceManager() = default;

void DeviceManager::addDevice(const std::shared_ptr<DeviceConfig> &config) {
    auto device_impl = std::make_unique<detail::DeviceImpl>(config);
    auto device = std::make_shared<Device>(std::move(device_impl));
    if (devices_.find(device->getName()) != devices_.end()) {
        return;
    }
    LOG_INFO("add device {}", device->getName());
    devices_[device->getName()] = device;
}

void DeviceManager::removeDevice(const std::string &device_name) {
    if (devices_.find(device_name) == devices_.end()) {
        LOG_ERROR("device {} not found", device_name);
        return;
    }
    devices_.erase(device_name);
}

Status DeviceManager::start() {
    bool all_started = true;
    LOG_INFO("start all devices");
    for (auto &[device_name, device] : devices_) {
        const auto status = device->start();
        if (status != Status::OK) {
            LOG_ERROR("failed to start device {}", device_name);
            all_started = false;
        }
    }
    LOG_INFO("start all devices done");
    return all_started ? Status::OK : Status::ERROR;
}

Status DeviceManager::stop() {
    LOG_INFO("stop all devices");
    bool all_stopped = true;
    for (auto &[device_name, device] : devices_) {
        const auto status = device->stop();
        if (status != Status::OK) {
            LOG_ERROR("failed to stop device {}", device_name);
            all_stopped = false;
        }
    }
    LOG_INFO("stop all devices done");
    return all_stopped ? Status::OK : Status::ERROR;
}

const std::map<std::string, std::shared_ptr<Device>>
    &DeviceManager::getDevices() const {
    return devices_;
}
}  // namespace ob_lidar_driver
