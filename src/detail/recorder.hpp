#pragma once

#include <memory>
#include <string>

#include "orbbec_lidar/device.hpp"
#include "orbbec_lidar/types.hpp"

namespace ob_lidar::detail {
class RecorderImpl {
   public:
    RecorderImpl(std::shared_ptr<Device> device, const std::string &output_dir);

    ~RecorderImpl();

    void start();

    void stop();

    void recordCallback(std::shared_ptr<Frame> frame);

    void enableRecordImu(bool enable);

    void enableRecordCommand(bool enable);

    void enableRecordPointCloud(bool enable);

   private:
    std::shared_ptr<Device> device_;
    std::string output_dir_;
};
}  // namespace ob_lidar::detail
