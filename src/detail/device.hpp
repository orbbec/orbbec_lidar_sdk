#pragma once

#include <spdlog/fmt/bundled/format.h>
#include <spdlog/spdlog.h>

#include <chrono>
#include <condition_variable>
#include <magic_enum/magic_enum.hpp>
#include <mutex>
#include <optional>
#include <orbbec_lidar/config.hpp>
#include <orbbec_lidar/frame.hpp>
#include <orbbec_lidar/option.hpp>
#include <orbbec_lidar/types.hpp>
#include <queue>

#include "MS600_capability.hpp"
#include "TL2401_capability.hpp"
#include "capability.hpp"
#include "frame.hpp"
#include "network_comm.hpp"
#include "option_mapper.hpp"
#include "request.hpp"
#include "response.hpp"
#include "spdlog/formatter.h"

namespace ob_lidar::detail {

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

    double getTemperatureScale();

    double getVoltageScale();

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

    Status setOption(const uint16_t &address, const void *value,
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

    Status getOption(const uint16_t &address, void *value, size_t value_size,
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
    void setCommunicationProtocol(const LidarProtocolType &protocol);

    void configureScanBlockSizeBasedOnWorkMode();

    Status sendCommandAndWaitForResponse(uint16_t command_id, const void *value,
                                         size_t value_size,
                                         CommandResponse &response);

    void imuDataCallback(const std::vector<uint8_t> &data);

    void logDataCallback(const std::vector<uint8_t> &data);

    void pointCloudDataCallback(const std::vector<uint8_t> &data);

    void spherePointCloudDataCallback(const std::vector<uint8_t> &data);

    void genericDataCallback(const uint8_t *data, size_t size);

    void onPointCloudDataReceived(const uint8_t *data, size_t size);

    void onScanDataReceived(const uint8_t *data, size_t size);

    void publishScanFrame(int frame_index,
                          const std::chrono::nanoseconds &timestamp,
                          const LaserScanMeta &meta_data,
                          const std::vector<uint16_t> &ranges,
                          const std::vector<uint16_t> &intensities);

    void extractScanData(const std::vector<uint8_t> &scan_data,
                         std::vector<uint16_t> &ranges,
                         std::vector<uint16_t> &intensities);

    LaserScanMeta createMS600LaserScanMeta(const MS600DataPacketHeader &header);

    void mergeScanData(std::vector<uint16_t> &ranges,
                       std::vector<uint16_t> &intensities);

    void clearScanDataQueue();

    void onCommandResponseReceived(const uint8_t *data, size_t size);

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
    std::unique_ptr<DataResponse> scan_data_packet_ = nullptr;
    // current scan speed, if you use hz, you can convert it to RPM, 1hz = 60RPM
    int scan_speed_ = 0;
    // max scan block size
    size_t max_scan_block_size_ = 0;
    // block point size
    size_t block_point_size_ = 0;
    // current block index
    int current_block_index_ = 0;
    // current scan
    std::queue<std::vector<uint8_t>> scan_blocks_;
    // sum of scan blocks size
    size_t sum_of_scan_blocks_size_ = 0;
    // wait all scan blocks
    bool wait_all_scan_blocks_ = true;
    // min angle
    double min_angle_ = 360;
    // max angle
    double max_angle_ = 0;
};

}  // namespace ob_lidar::detail
