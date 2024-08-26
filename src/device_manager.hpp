#pragma once
#include "orb_lidar_driver/device.hpp"

namespace ob_lidar_driver {

class DeviceManager {
   public:
    DeviceManager();

    ~DeviceManager();

    void addDevice(const std::shared_ptr<DeviceConfig> &config);

    void removeDevice(const std::string &device_name);

    Status start();

    Status stop();

    [[nodiscard]] const std::map<std::string, std::shared_ptr<Device>>
        &getDevices() const;

   private:
    std::map<std::string, std::shared_ptr<Device>> devices_;
};
}  // namespace ob_lidar_driver
