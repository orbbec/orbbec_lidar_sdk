#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "device.hpp"
namespace ob_lidar_driver {
namespace detail {
class RecorderImpl;
}  // namespace detail

class Recorder {
   public:
    /**
     * @class Recorder
     * @brief The Recorder class is responsible for recording data from a Lidar
     * device.
     *
     * The Recorder class provides functionality to record data from a Lidar
     * device and save it to a specified output directory.
     */
    Recorder(std::shared_ptr<Device> device, const std::string &output_dir);

    ~Recorder();

    /**
     * @brief Start recording data from the Lidar device.
     */
    void start() const;

    /**
     * @brief Stop recording data from the Lidar device.
     */
    void stop();

    /**
     * @brief Enable recording IMU data from the Lidar device.
     *
     * @param enable Whether to enable recording IMU data.
     */
    void enableRecordImu(bool enable) const;

    /**
     * @brief Enable recording command data from the Lidar device.
     *
     * @param enable Whether to enable recording command data.
     */
    void enableRecordCommand(bool enable) const;

    /**
     * @brief Enable recording point cloud data from the Lidar device.
     *
     * @param enable Whether to enable recording point cloud data.
     */
    void enableRecordPointCloud(bool enable) const;

   private:
    std::unique_ptr<detail::RecorderImpl> impl_;
};
}  // namespace ob_lidar_driver
