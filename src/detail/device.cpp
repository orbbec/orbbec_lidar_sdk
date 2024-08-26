#include "device.hpp"

#include <chrono>
#include <memory>
#include <set>
#include <utils.hpp>

#include "../logger.hpp"
#include "check.hpp"
#include "frame.hpp"
#include "orb_lidar_driver/frame.hpp"

namespace ob_lidar_driver::detail {
DeviceImpl::DeviceImpl(const std::string& config_file)
    : config_(std::make_shared<DeviceConfig>(config_file)) {
    init();
}

DeviceImpl::DeviceImpl(const std::shared_ptr<DeviceConfig>& config)
    : config_(config) {
    init();
}

void DeviceImpl::loadDeviceInfo() {
    // get serial number
    char serial_number[128];
    constexpr auto serial_number_option =
        LidarOption::OB_LIDAR_OPTION_SERIAL_NUMBER;
    size_t size_read = 0;
    getOption(serial_number_option, serial_number, sizeof(serial_number),
              &size_read);
    serial_number[size_read] = '\0';
    device_info_.serial_number = serial_number;

    // get firmware version
    char firmware_version[128];
    constexpr auto firmware_version_option =
        LidarOption::OB_LIDAR_OPTION_FIRMWARE_VERSION;
    size_read = 0;
    getOption(firmware_version_option, firmware_version,
              sizeof(firmware_version), &size_read);
    device_info_.firmware_version = firmware_version;
    // get fpga version
    char fpga_version[128];
    constexpr auto fpga_version_option =
        LidarOption::OB_LIDAR_OPTION_FPGA_VERSION;
    size_read = 0;
    getOption(fpga_version_option, fpga_version, sizeof(fpga_version),
              &size_read);
    fpga_version[size_read] = '\0';
    device_info_.fpga_version = fpga_version;
}

void DeviceImpl::registerDataCallbacks() {
    CHECK_NOTNULL(network_config_);
    if (network_config_->isSinglePort()) {
        LOG_INFO("Registering data callback for single port");
        network_comm_->setOnDataCallback(
            [this](const std::vector<uint8_t>& data) {
                genericDataCallback(data);
            });
    } else {
        // command channel
        LOG_INFO("Registering data callback for command channel");
        network_comm_->setOnDataCallback(
            LidarChannelType::COMMAND,
            [this](const std::vector<uint8_t>& data) {
                genericDataCallback(data);
            });
        // TODO: point cloud channel
        // TODO: sphere point cloud channel
        // TODO: imu channel
        // TODO: log channel
    }
}

Status DeviceImpl::connect() {
    LOG_INFO("Connecting to device: {}", device_name_);
    constexpr auto connection_option =
        LidarOption::OB_LIDAR_OPTION_INITIATE_DEVICE_CONNECTION;
    const auto command_id = static_cast<uint16_t>(
        LidarOptionMapper::getSetCommandInterface(connection_option));
    return setOption(connection_option, &command_id, sizeof(command_id));
}

DeviceImpl::~DeviceImpl() {
    LOG_INFO("Destroying device: {}", device_name_);
    if (is_started_) {
        stop();
    }
    LOG_INFO("Device {} destroyed", device_name_);
}

void DeviceImpl::init() {
    CHECK_NOTNULL(config_);
    device_name_ = config_->getDeviceName();
    LOG_INFO("Initializing device: {}", device_name_);
    model_ = config_->getModel();
    device_info_.device_name = device_name_;
    device_info_.model = magic_enum::enum_name(model_);
    network_config_ = config_->getNetworkConfig();
    CHECK_NOTNULL(network_config_);
    network_comm_ = NetworkCommFactory::create(config_->getNetworkConfig());

    if (model_ == LidarModel::MS600) {
        capability_ = std::make_shared<MS600LidarCapabilities>();
    } else if (model_ == LidarModel::TL2401) {
        capability_ = std::make_shared<TL2401LidarCapabilities>();
    } else {
        throw std::runtime_error("Unsupported lidar model");
    }
    registerDataCallbacks();
    auto status = connect();
    if (status == Status::OK) {
        LOG_INFO("Connected to device: {}", device_name_);
    } else {
        LOG_ERROR("Failed to connect to device: {}", device_name_);
    }
    loadDeviceInfo();
    LOG_INFO("Device {} initialized", device_name_);
}

DeviceInfo DeviceImpl::getInfo() { return device_info_; }

Status DeviceImpl::start() {
    if (!frame_callback_) {
        LOG_ERROR("Frame callback is not set");
        return Status::ERROR;
    }
    LOG_INFO("Starting device: {}", device_name_);
    constexpr auto streaming_option = OB_LIDAR_OPTION_ENABLE_STREAMING;
    constexpr int enable_streaming = 1;
    // TODO: set stream config, such as frame rate, scan direction, etc.
    const auto status = setOption(streaming_option, &enable_streaming,
                                  sizeof(enable_streaming));
    if (status == Status::OK) {
        is_started_ = true;
        LOG_INFO("Device {} started", device_name_);
    } else {
        LOG_ERROR("Failed to start device: {}", device_name_);
    }
    return status;
}

Status DeviceImpl::stop() {
    if (!is_started_) {
        LOG_INFO("Device {} is not started", device_name_);
        return Status::OK;
    }
    constexpr auto streaming_option = OB_LIDAR_OPTION_ENABLE_STREAMING;
    constexpr int disable_streaming = 0;
    const auto status = setOption(streaming_option, &disable_streaming,
                                  sizeof(disable_streaming));
    if (status == Status::OK) {
        is_started_ = false;
        LOG_INFO("Device {} stopped", device_name_);
    } else {
        LOG_ERROR("Failed to stop device: {}", device_name_);
    }
    return status;
}

Status DeviceImpl::start(const std::shared_ptr<StreamConfig>& config,
                         const FrameCallback& callback) {
    frame_callback_ = callback;
    // TODO: set stream config
    return start();
}

void DeviceImpl::registerFrameObserver(int observer_id,
                                       const FrameCallback& callback) {
    frame_observers_[observer_id] = callback;
}

void DeviceImpl::unregisterFrameObserver(int observer_id) {
    if (frame_observers_.find(observer_id) != frame_observers_.end()) {
        frame_observers_.erase(observer_id);
    }
}

bool DeviceImpl::isOptionSupported(const LidarOption& option) {
    return capability_->isOptionSupported(option);
}

bool DeviceImpl::isOptionSupported(const LidarOption& type,
                                   const LidarOptionPermission& permission) {
    return capability_->isOptionSupported(type, permission);
}

bool DeviceImpl::isStreamSupported(const LidarStreamType& type) const {
    return capability_->isStreamSupported(type);
}

LidarModel DeviceImpl::getModel() const { return model_; }

std::string DeviceImpl::getName() const { return device_name_; }

void DeviceImpl::setDeviceName(const std::string& name) {
    device_name_ = name;
    device_info_.device_name = name;
}

Status DeviceImpl::sendCommandAndWaitForResponse(uint16_t command_id,
                                                 const void* value,
                                                 size_t value_size,
                                                 CommandResponse& response) {
    Request request(command_id);
    auto payload = request.serialize(value, value_size);

    // Clear command queue
    {
        std::unique_lock lock(command_data_queue_mutex_);
        while (!command_data_queue_.empty()) {
            command_data_queue_.pop();
        }
    }
    // Send data
    {
        std::unique_lock lock(network_comm_mutex_);
        CHECK_NOTNULL(network_comm_);
        CHECK_NOTNULL(network_config_);
        if (network_config_->isSinglePort()) {
            network_comm_->sendData(payload);
        } else {
            network_comm_->sendData(LidarChannelType::COMMAND, payload);
        }
    }
    // Wait for response
    {
        std::unique_lock lock(command_data_queue_mutex_);
        const auto timeout = config_->getOptionTimeout();
        if (!command_data_queue_cv_.wait_for(lock, timeout, [this]() {
                return !command_data_queue_.empty();
            })) {
            LOG_ERROR("Timeout waiting for response {}",
                      commandIDToString(command_id));
            return Status::TIMEOUT;
        }

        response = command_data_queue_.front();
        command_data_queue_.pop();
    }
    if (!response.isValid() || response.getCommandId() != command_id ||
        response.getStatus() != 0) {
        LOG_ERROR("Invalid response");
        return Status::INVALID;
    }

    return Status::OK;
}

Status DeviceImpl::setOption(const LidarOption& option, const void* value,
                             size_t value_size) {
    auto command_interface = LidarOptionMapper::getSetCommandInterface(option);
    if (command_interface == LidarCommandInterface::UNKNOWN) {
        LOG_ERROR("Unknown option: {}", magic_enum::enum_name(option));
        return Status::ERROR;
    }
    auto command_id = static_cast<uint16_t>(command_interface);
    CommandResponse response;
    auto status =
        sendCommandAndWaitForResponse(command_id, value, value_size, response);
    return status;
}

Status DeviceImpl::getOption(const LidarOption& option, void* value,
                             size_t value_size, size_t* size_read) {
    const auto command_interface =
        LidarOptionMapper::getGetCommandInterface(option);
    if (command_interface == LidarCommandInterface::UNKNOWN) {
        LOG_ERROR("Unknown option: {}", magic_enum::enum_name(option));
        return Status::ERROR;
    }
    const auto command_id = static_cast<uint16_t>(command_interface);
    CommandResponse response;
    const auto status =
        sendCommandAndWaitForResponse(command_id, value, value_size, response);

    if (status == Status::OK) {
        response.getData(value, value_size, size_read);
    } else {
    }

    return status;
}

void DeviceImpl::imuDataCallback(const std::vector<uint8_t>& data) {
    // TODO: parse imu data
    // TODO: callback to frame callback
}

void DeviceImpl::logDataCallback(const std::vector<uint8_t>& data) {
    // TODO: parse log data
    // TODO: callback to frame callback
}

void DeviceImpl::pointCloudDataCallback(const std::vector<uint8_t>& data) {
    // TODO: parse point cloud data
    // TODO: callback to frame callback
}

void DeviceImpl::spherePointCloudDataCallback(
    const std::vector<uint8_t>& data) {
    // TODO: parse sphere point cloud data
    // TODO: callback to frame callback
}

#define COMMAND_TYPE_RESPONSE_HEADER 0xFE01
#define DATA_TYPE_RESPONSE_HEADER 0x534D

void DeviceImpl::genericDataCallback(const std::vector<uint8_t>& data) {
    uint16_t header;
    if (data.size() < 2) {
        LOG_ERROR("Data size is too small");
        return;
    }
    memcpy(&header, data.data(), sizeof(uint16_t));
    header = ntohs(header);
    if (header == COMMAND_TYPE_RESPONSE_HEADER) {
        std::unique_lock lock(command_data_queue_mutex_);
        CommandResponse response;
        response.setData(data);
        command_data_queue_.push(response);
        command_data_queue_cv_.notify_all();
    } else if (header == DATA_TYPE_RESPONSE_HEADER) {
        DataResponse response(model_);
        response.setData(data);
        const auto& point_data = response.getData();
        auto data_type = response.getDataType();
        auto frame_index = response.getFrameIndex();
        auto lidar_frame_type = static_cast<LidarFrameType>(data_type);
        if (lidar_frame_type == LidarFrameType::POINT_CLOUD ||
            lidar_frame_type == LidarFrameType::SPHERE_POINT_CLOUD) {
            std::unique_ptr<PointCloudFrameImpl> frame_impl =
                std::make_unique<PointCloudFrameImpl>();
            CHECK_NOTNULL(frame_impl);
            const auto timestamp =
                std::chrono::nanoseconds(response.getTimestamp());
            frame_impl->setData(point_data.data(), point_data.size());
            frame_impl->setTimestamp(timestamp);
            frame_impl->setFrameType(lidar_frame_type);
            // TODO: Remove hardcoded distance and angle scale
            frame_impl->setDistanceScale(2.0);
            frame_impl->setAngleScale(0.01);
            frame_impl->setFrameId(frame_index);
            std::shared_ptr<Frame> frame =
                std::make_shared<PointCloudFrame>(std::move(frame_impl));
            if (frame_callback_) {
                frame_callback_(frame);
            }
            for (const auto& [_, observer] : frame_observers_) {
                observer(frame);
            }
        } else {
        }
    }
}
}  // namespace ob_lidar_driver::detail
