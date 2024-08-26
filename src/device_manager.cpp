#include "orbbec_lidar/device_manager.hpp"

#include "detail/device_manager.hpp"

namespace ob_lidar {
DeviceManager::DeviceManager(const std::string &config_file_path)
    : impl_(std::make_unique<detail::DeviceManagerImpl>(config_file_path)) {}

void DeviceManager::enableDiscovery(bool enable) { impl_->enableDiscovery(enable); }

void DeviceManager::setOnDeviceChangedCallback(
    const onDeviceConnectedCallback &on_device_connected,
    const onDeviceDisconnectedCallback &on_device_disconnected) const {
    impl_->setOnDeviceChangedCallback(on_device_connected,
                                      on_device_disconnected);
}

DeviceManager::DeviceManager() : impl_(std::make_unique<detail::DeviceManagerImpl>()) {}

DeviceManager::~DeviceManager() = default;

std::vector<std::shared_ptr<Device>> DeviceManager::getDevices() {
    return impl_->getDevices();
}

std::shared_ptr<Device> DeviceManager::getDevice(const std::string &device_name) {
    return impl_->getDevice(device_name);
}

void DeviceManager::addDevice(std::shared_ptr<DeviceConfig> config) {
    impl_->addDevice(config);
}

void DeviceManager::removeDevice(const std::string &device_name) {
    impl_->removeDevice(device_name);
}
void DeviceManager::start() { impl_->start(); }

void DeviceManager::stop() { impl_->stop(); }
}  // namespace ob_lidar
