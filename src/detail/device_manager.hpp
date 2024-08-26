#pragma once

#include <spdlog/spdlog.h>

#include <memory>
#include <vector>

#include "../logger.hpp"
#include "orbbec_lidar/config.hpp"
#include "orbbec_lidar/device.hpp"

namespace ob_lidar::detail {

class DeviceManagerImpl {
   public:
    explicit DeviceManagerImpl(const std::string &config_file);

    void enableDiscovery(bool enable);

    void setOnDeviceChangedCallback(
        const onDeviceConnectedCallback &on_device_connected,
        const onDeviceDisconnectedCallback &on_device_disconnected);

    DeviceManagerImpl();

    ~DeviceManagerImpl();

    std::vector<std::shared_ptr<Device>> getDevices();

    std::shared_ptr<Device> getDevice(const std::string &device_name);

    void addDevice(std::shared_ptr<DeviceConfig> config);

    void removeDevice(const std::string &device_name);

    void start();

    void stop();

    static std::shared_ptr<Logger> logger_;

    [[nodiscard]] bool isDiscoveryEnabled() const;

    void discovery();

   private:
    std::map<std::string, std::shared_ptr<Device>> devices_;
    onDeviceConnectedCallback on_device_connected_;
    onDeviceDisconnectedCallback on_device_disconnected_;
    bool discovery_enabled_ = false;
    std::shared_ptr<std::thread> discovery_thread_ = nullptr;
    std::mutex devices_mutex_;
    std::condition_variable devices_cv_;
    std::atomic<bool> stop_discovery_{false};
};

}  // namespace ob_lidar::detail
