#include "detail/device.hpp"

#include <orbbec_lidar/device.hpp>
#include <uvw.hpp>

namespace ob_lidar {

Device::Device(std::unique_ptr<detail::DeviceImpl> impl)
    : impl_(std::move(impl)) {}

Device::~Device() noexcept = default;

DeviceInfo Device::getInfo() { return impl_->getInfo(); }

Status Device::start() { return impl_->start(); }

Status Device::stop() { return impl_->stop(); }

Status Device::start(const std::shared_ptr<StreamConfig> &config,
                     const FrameCallback &callback) {
    return impl_->start(config, callback);
}

void Device::registerFrameObserver(int observer_id,
                                   const FrameCallback &callback) {
    impl_->registerFrameObserver(observer_id, callback);
}

void Device::unregisterFrameObserver(int observer_id) {
    impl_->unregisterFrameObserver(observer_id);
}

Status Device::setIntOption(const LidarOption &option, int value) {
    // convert to network order
    return impl_->setOption<int>(option, htonl(value));
}

Status Device::setUint16Option(const LidarOption &option, uint16_t value) {
    // convert to network order
    return impl_->setOption<uint16_t>(option, htons(value));
}

Status Device::setBoolOption(const LidarOption &option, bool value) {
    return impl_->setOption<bool>(option, value);
}

Status Device::setFloatOption(const LidarOption &option, float value) {
    return impl_->setOption<float>(option, value);
}

Status Device::setOption(const LidarOption &option, const void *value,
                         size_t value_size) {
    return impl_->setOption(option, value, value_size);
}

Status Device::setOption(const uint16_t &address, const void *value,
                         size_t value_size) {
    return impl_->setOption(address, value, value_size);
}

int Device::getIntOption(const LidarOption &option) {
    // convert to host order
    return ntohl(impl_->getOption<int>(option));
}

uint16_t Device::getUint16Option(const LidarOption &option) {
    // convert to host order
    return ntohs(impl_->getOption<uint16_t>(option));
}

bool Device::getBoolOption(const LidarOption &option) {
    return impl_->getOption<bool>(option);
}

float Device::getFloatOption(const LidarOption &option) {
    return impl_->getOption<float>(option);
}

Status Device::getOption(const LidarOption &option, void *value,
                         size_t value_size, size_t *size_read) {
    return impl_->getOption(option, value, value_size, size_read);
}

Status Device::getOption(const uint16_t &address, void *value, size_t size,
                         size_t *size_read) {
    return impl_->getOption(address, value, size, size_read);
}

bool Device::isOptionSupported(const LidarOption &option) {
    return impl_->isOptionSupported(option);
}

bool Device::isOptionSupported(const LidarOption &type,
                               const LidarOptionPermission &permission) {
    return impl_->isOptionSupported(type, permission);
}

bool Device::isStreamSupported(const LidarStreamType &type) {
    return impl_->isStreamSupported(type);
}

LidarModel Device::getModel() { return impl_->getModel(); }

std::string Device::getName() { return impl_->getName(); }

}  // namespace ob_lidar
