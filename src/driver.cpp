#include "orb_lidar_driver/driver.hpp"

#include "detail/driver.hpp"

namespace ob_lidar_driver {
Driver::Driver(const std::string &config_file_path)
    : impl_(std::make_unique<detail::DriverImpl>(config_file_path)) {}

void Driver::enableDiscovery(bool enable) { impl_->enableDiscovery(enable); }

void Driver::setOnDeviceChangedCallback(
    const onDeviceConnectedCallback &on_device_connected,
    const onDeviceDisconnectedCallback &on_device_disconnected) const {
    impl_->setOnDeviceChangedCallback(on_device_connected,
                                      on_device_disconnected);
}

Driver::Driver() : impl_(std::make_unique<detail::DriverImpl>()) {}

Driver::~Driver() = default;

std::vector<std::shared_ptr<Device>> Driver::getDevices() {
    return impl_->getDevices();
}

std::shared_ptr<Device> Driver::getDevice(const std::string &device_name) {
    return impl_->getDevice(device_name);
}

void Driver::addDevice(std::shared_ptr<DeviceConfig> config) {
    impl_->addDevice(config);
}

void Driver::removeDevice(const std::string &device_name) {
    impl_->removeDevice(device_name);
}
void Driver::start() { impl_->start(); }

void Driver::stop() { impl_->stop(); }
}  // namespace ob_lidar_driver
