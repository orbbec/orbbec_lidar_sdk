#pragma once

#include <spdlog/spdlog.h>

#include <chrono>
#include <condition_variable>
#include <magic_enum/magic_enum.hpp>
#include <mutex>
#include <optional>
#include <orb_lidar_driver/config.hpp>
#include <orb_lidar_driver/frame.hpp>
#include <orb_lidar_driver/option.hpp>
#include <orb_lidar_driver/types.hpp>
#include <queue>

#include "MS600_capability.hpp"
#include "TL2401_capability.hpp"
#include "capability.hpp"
#include "network_comm.hpp"
#include "option_mapper.hpp"
#include "request.hpp"
#include "response.hpp"
#include "spdlog/formatter.h"

namespace ob_lidar_driver::detail {

class DeviceImpl {
   public:
    explicit DeviceImpl(const std::string &config_file);

    explicit DeviceImpl(const std::shared_ptr<DeviceConfig> &config);

    void init();

    void loadDeviceInfo();

    Status connect();

    void registerDataCallbacks();

    ~DeviceImpl() noexcept;

    DeviceInfo getInfo();

    Status start();

    Status stop();

    Status start(const std::shared_ptr<StreamConfig> &config,
                 const FrameCallback &callback);

    void registerFrameObserver(int observer_id, const FrameCallback &callback);

    void unregisterFrameObserver(int observer_id);

    bool isOptionSupported(const LidarOption &option);

    bool isOptionSupported(const LidarOption &type,
                           const LidarOptionPermission &permission);

    bool isStreamSupported(const LidarStreamType &type) const;

    [[nodiscard]] LidarModel getModel() const;

    [[nodiscard]] std::string getName() const;

    void setDeviceName(const std::string &name);

    Status setOption(const LidarOption &option, const void *value,
                     size_t value_size);

    template <typename T>
    Status setOption(const LidarOption &option, const T &value) {
        if (!isOptionSupported(option)) {
            return Status::ERROR;
        }
        if constexpr (std::is_pod_v<T>) {
            return setOption(option, &value, sizeof(value));
        } else if constexpr (std::is_same_v<T, std::string>) {
            return setOption(option, value.c_str(), value.size());
        } else {
            return Status::ERROR;
        }
    }

    Status getOption(const LidarOption &option, void *value, size_t value_size,
                     size_t *size_read);

    template <typename T>
    T getOption(const LidarOption &option) {
        // Check if the option is supported
        if (!isOptionSupported(option)) {
            const auto err_msg = spdlog::fmt_lib::format(
                "Option {} is not supported", magic_enum::enum_name(option));
            throw std::runtime_error(err_msg);
        }
        if constexpr (std::is_pod_v<T>) {
            T value;
            size_t size_read = 0;
            getOption(option, &value, sizeof(value), &size_read);
            return value;
        } else {
            const auto err_msg = spdlog::fmt_lib::format(
                "Option {} is not supported", magic_enum::enum_name(option));
            throw std::runtime_error(err_msg);
        }
    }

   private:
    Status sendCommandAndWaitForResponse(uint16_t command_id, const void *value,
                                         size_t value_size,
                                         CommandResponse &response);

    void imuDataCallback(const std::vector<uint8_t> &data);

    void logDataCallback(const std::vector<uint8_t> &data);

    void pointCloudDataCallback(const std::vector<uint8_t> &data);

    void spherePointCloudDataCallback(const std::vector<uint8_t> &data);

    void genericDataCallback(const std::vector<uint8_t> &data);

    // device name unique
    std::string device_name_;
    // product model
    LidarModel model_ = LidarModel::UNKNOWN;
    // device config
    std::shared_ptr<DeviceConfig> config_ = nullptr;
    // network config
    std::shared_ptr<NetworkConfig> network_config_ = nullptr;
    std::atomic_bool is_started_{false};
    // device info
    DeviceInfo device_info_;
    // frame callback
    FrameCallback frame_callback_;
    // network communication
    std::unique_ptr<NetworkComm> network_comm_ = nullptr;
    std::mutex network_comm_mutex_;
    // lidar capabilities
    std::shared_ptr<LidarCapabilitiesInterface> capability_ = nullptr;
    // frame observers
    std::map<int, FrameCallback> frame_observers_;
    // command data queue
    std::queue<CommandResponse> command_data_queue_;
    // command data queue mutex
    std::mutex command_data_queue_mutex_;
    // command data queue condition variable
    std::condition_variable command_data_queue_cv_;
};

}  // namespace ob_lidar_driver::detail
