#include "orbbec_lidar/recorder.hpp"

#include "detail/recorder.hpp"

namespace ob_lidar {

Recorder::Recorder(std::shared_ptr<Device> device,
                   const std::string &output_dir)
    : impl_(std::make_unique<detail::RecorderImpl>(device, output_dir)) {}

Recorder::~Recorder() = default;

void Recorder::start() const { impl_->start(); }

void Recorder::enableRecordCommand(bool enable) const {
    impl_->enableRecordCommand(enable);
}

void Recorder::enableRecordPointCloud(bool enable) const {
    impl_->enableRecordPointCloud(enable);
}

void Recorder::enableRecordImu(bool enable) const {
    impl_->enableRecordImu(enable);
}

}  // namespace ob_lidar
