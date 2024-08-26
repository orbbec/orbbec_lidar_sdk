// macp see https://github.com/foxglove/mcap/tree/main/cpp
#define MCAP_IMPLEMENTATION
#include "recorder.hpp"

#include <mcap/writer.hpp>

namespace ob_lidar_driver::detail {

RecorderImpl::RecorderImpl(std::shared_ptr<Device> device,
                           const std::string &output_dir)
    : device_(device), output_dir_(output_dir) {}

RecorderImpl::~RecorderImpl() {}

void RecorderImpl::start() {}

void RecorderImpl::stop() {}

void RecorderImpl::recordCallback(const std::shared_ptr<Frame> frame) {}

void RecorderImpl::enableRecordImu(bool enable) {}

void RecorderImpl::enableRecordCommand(bool enable) {}

void RecorderImpl::enableRecordPointCloud(bool enable) {}

}  // namespace ob_lidar_driver::detail
