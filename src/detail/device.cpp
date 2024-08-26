#include "device.hpp"

#include <chrono>
#include <memory>
#include <set>
#include <utils.hpp>

#include "../logger.hpp"
#include "check.hpp"
#include "frame.hpp"
#include "orbbec_lidar/frame.hpp"
#include "spdlog/sinks/null_sink.h"

namespace ob_lidar::detail {
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
    char serial_number[64];
    constexpr auto serial_number_option =
        LidarOption::OB_LIDAR_OPTION_SERIAL_NUMBER;
    size_t size_read = 0;
    getOption(serial_number_option, serial_number, sizeof(serial_number),
              &size_read);
    serial_number[size_read] = '\0';
    device_info_.serial_number = serial_number;
    LOG_INFO("Serial number: {}", device_info_.serial_number);

    // get firmware version
    char firmware_version[64];
    constexpr auto firmware_version_option =
        LidarOption::OB_LIDAR_OPTION_FIRMWARE_VERSION;
    size_read = 0;
    getOption(firmware_version_option, firmware_version,
              sizeof(firmware_version), &size_read);
    device_info_.firmware_version = firmware_version;
    LOG_INFO("Firmware version: {}", device_info_.firmware_version);
    // get fpga version
    char fpga_version[64];
    constexpr auto fpga_version_option =
        LidarOption::OB_LIDAR_OPTION_FPGA_VERSION;
    size_read = 0;
    getOption(fpga_version_option, fpga_version, sizeof(fpga_version),
              &size_read);
    fpga_version[size_read] = '\0';
    device_info_.fpga_version = fpga_version;
    LOG_INFO("FPGA version: {}", device_info_.fpga_version);
}

void DeviceImpl::registerDataCallbacks() {
    CHECK_NOTNULL(network_config_);
    if (network_config_->isSinglePort()) {
        LOG_INFO("Registering data callback for single port");
        network_comm_->setOnDataCallback(
            [this](const uint8_t* data, size_t size) {
                genericDataCallback(data, size);
            });
    } else {
        // command channel
        LOG_INFO("Registering data callback for command channel");
        network_comm_->setOnDataCallback(
            LidarChannelType::COMMAND,
            [this](const uint8_t* data, size_t size) {
                genericDataCallback(data, size);
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
    uint32_t command_code = htonl(0x12345678);
    return setOption(connection_option, &command_code, sizeof(command_code));
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
    CHECK_NOTNULL(network_comm_);
    // setCommunicationProtocol(network_config_->getProtocolType());

    if (model_ == LidarModel::MS600) {
        capability_ = std::make_shared<MS600LidarCapabilities>();
        scan_data_packet_ = std::make_unique<DataResponse>(model_);
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
    if (model_ == LidarModel::MS600) {
        configureScanBlockSizeBasedOnWorkMode();
    }
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
    const int enable_streaming = htonl(1);
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
    const int disable_streaming = htonl(0);
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
    // FIXME: need handle IMU and log stream in the future
    auto stream_list = {LidarStreamType::SCAN,
                        LidarStreamType::SPHERE_POINT_CLOUD,
                        LidarStreamType::POINT_CLOUD};
    bool scan_stream_set = false;
    // get scan speed
    int scan_speed = 0;
    size_t size_read = 0;
    getOption(OB_LIDAR_OPTION_SCAN_SPEED, &scan_speed, sizeof(scan_speed),
              &size_read);
    scan_speed = ntohl(scan_speed);
    LOG_INFO("Scan speed: {} RPM", scan_speed);
    for (const auto& stream : stream_list) {
        if (config->isStreamEnabled(stream)) {
            double freq = config->getStreamFrequency(stream);
            if (capability_->isStreamFreqSupported(stream, freq)) {
                int value = htonl(static_cast<int>(freq * 60));
                if (freq * 60 == scan_speed) {
                    scan_stream_set = true;
                    break;
                }
                auto status = setOption(OB_LIDAR_OPTION_SCAN_SPEED, &value,
                                        sizeof(value));
                if (status != Status::OK) {
                    LOG_ERROR("Failed to set scan speed: {}", freq);
                    return status;
                }
                // if find the first supported stream, break
                scan_stream_set = true;
                break;
            }
        }
    }
    if (!scan_stream_set) {
        LOG_ERROR("No supported scan stream found");
        return Status::ERROR;
    }
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
        const std::chrono::seconds timeout(1);
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
        LOG_ERROR("{} get invalid response: {}", commandIDToString(command_id),
                  response.getStatus());
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
    // if is set work mode option, set max block size.
    if (option == LidarOption::OB_LIDAR_OPTION_WORK_MODE &&
        model_ == LidarModel::MS600) {
        auto work_mode = *static_cast<const uint32_t*>(value);
        if (work_mode == 3) {
            // I don't know why, but this is the max scan block size for 20hz
            max_scan_block_size_ = 90;
            block_point_size_ = LASER_SCAN_BLOCK_LENGTH_20HZ_CALIB;
        } else {
            // 10hz
            max_scan_block_size_ = 18;
            block_point_size_ = LASER_SCAN_BLOCK_LENGTH_20HZ;
        }
    }
    return status;
}

Status DeviceImpl::setOption(const uint16_t& address, const void* value,
                             size_t value_size) {
    CommandResponse response;
    auto status =
        sendCommandAndWaitForResponse(address, value, value_size, response);
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
        // TODO: handle error
    }

    return status;
}

Status DeviceImpl::getOption(const uint16_t& address, void* value,
                             size_t value_size, size_t* size_read) {
    CommandResponse response;
    const auto status =
        sendCommandAndWaitForResponse(address, value, value_size, response);

    if (status == Status::OK) {
        response.getData(value, value_size, size_read);
    } else {
        // TODO: handle error
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

void DeviceImpl::genericDataCallback(const uint8_t* data, size_t size) {
    uint16_t header;
    if (size < 2) {
        LOG_ERROR("Data size is too small");
        return;
    }
    memcpy(&header, data, sizeof(uint16_t));
    // header = ntohs(header);
    if (header == COMMAND_TYPE_RESPONSE_HEADER) {
        onCommandResponseReceived(data, size);
    } else if (header == DATA_TYPE_RESPONSE_HEADER &&
               model_ == LidarModel::TL2401) {
        onPointCloudDataReceived(data, size);
    } else if (header == DATA_TYPE_RESPONSE_HEADER &&
               model_ == LidarModel::MS600) {
        onScanDataReceived(data, size);
    }
}

void DeviceImpl::onPointCloudDataReceived(const uint8_t* data, size_t size) {
    DataResponse response(model_);
    response.setData(data, size);
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
        frame_impl->setFrameId(frame_index);
        // TODO: Remove hardcoded distance and angle scale
        PointCloudMeta meta_data{};
        meta_data.distance_scale = 0.002;
        meta_data.angle_scale = 0.01;
        frame_impl->setMeta(meta_data);
        std::shared_ptr<Frame> frame =
            std::make_shared<PointCloudFrame>(std::move(frame_impl));
        if (frame_callback_) {
            frame_callback_(frame);
        }
        for (const auto& [_, observer] : frame_observers_) {
            observer(frame);
        }
    }
}

void DeviceImpl::onScanDataReceived(const uint8_t* data, size_t size) {
    if (!is_started_) {
        return;
    }
    if (model_ != LidarModel::MS600) {
        LOG_ERROR("Only model MS600 is supported scan data");
        return;
    }
    if(scan_data_packet_ == nullptr) {
        LOG_ERROR("Scan data packet is not initialized");
        return;
    }

    // Parse the incoming data
    scan_data_packet_->setData(data, size);
    auto frame_index = scan_data_packet_->getFrameIndex();
    const auto timestamp = std::chrono::nanoseconds(scan_data_packet_->getTimestamp());
    auto scan_data = scan_data_packet_->getData();

    // Accumulate the size of scan blocks

    // Get the header information from the response
    auto header = std::get<MS600DataPacketHeader>(scan_data_packet_->getHeader());
    uint8_t block_index = header.block_index;

    // Use angle scaling to calculate actual angles
    sum_of_scan_blocks_size_ += scan_data.size();
    if (!wait_all_scan_blocks_) {
        // If not waiting for all scan blocks, publish the scan data immediately
        if (scan_data.size() % 4 != 0) {
            LOG_ERROR("Invalid scan data size: {}", scan_data.size());
            return;
        }
        min_angle_ = header.start_angle * 0.01;
        max_angle_ = header.end_angle * 0.01;
        LaserScanMeta meta_data = createMS600LaserScanMeta(header);
        std::vector<uint16_t> ranges, intensities;

        extractScanData(scan_data, ranges, intensities);

        publishScanFrame(frame_index, timestamp, meta_data, ranges,
                         intensities);
        sum_of_scan_blocks_size_ = 0;  // Reset total size of scan blocks
        return;
    }
    min_angle_ = std::min(min_angle_, header.start_angle * 0.01);
    max_angle_ = std::max(max_angle_, header.end_angle * 0.01);

    // Push the scan data block into the queue
    scan_blocks_.push(scan_data);

    // Ensure that the maximum scan block size is valid
    CHECK(max_scan_block_size_ > 0);

    // If we reached the last block, process and merge the data
    if (block_index == max_scan_block_size_) {
        // Ensure that the accumulated size is divisible by 4 (each scan point
        // takes 4 bytes)
        if (sum_of_scan_blocks_size_ % 4 != 0) {
            LOG_ERROR("Invalid scan data size: {}", sum_of_scan_blocks_size_);
            sum_of_scan_blocks_size_ = 0;
            clearScanDataQueue();
            return;
        }

        LaserScanMeta meta_data = createMS600LaserScanMeta(header);
        std::vector<uint16_t> ranges, intensities;
        mergeScanData(ranges, intensities);
        meta_data.scan_size = ranges.size();
        publishScanFrame(frame_index, timestamp, meta_data, ranges,
                         intensities);
        sum_of_scan_blocks_size_ = 0;  // Reset total size of scan blocks
    }
}

void DeviceImpl::publishScanFrame(int frame_index,
                                  const std::chrono::nanoseconds& timestamp,
                                  const LaserScanMeta& meta_data,
                                  const std::vector<uint16_t>& ranges,
                                  const std::vector<uint16_t>& intensities) {
    auto frame_impl = std::make_unique<ScanFrameImpl>(ranges.size() * 4);
    frame_impl->setTimestamp(timestamp);
    frame_impl->setFrameType(LidarFrameType::SCAN);
    frame_impl->setFrameId(frame_index);
    frame_impl->setRanges(ranges);
    frame_impl->setIntensities(intensities);
    frame_impl->setMeta(meta_data);

    std::shared_ptr<Frame> frame =
        std::make_shared<ScanFrame>(std::move(frame_impl));
    if (frame_callback_) {
        frame_callback_(frame);
    }
    for (const auto& [_, observer] : frame_observers_) {
        observer(frame);
    }
}

LaserScanMeta DeviceImpl::createMS600LaserScanMeta(
    const MS600DataPacketHeader& header) {
    LaserScanMeta meta_data{};
    meta_data.sync_mode = header.sync_mode;
    meta_data.scan_size = header.frame_length;
    meta_data.start_angle = min_angle_;
    meta_data.end_angle = max_angle_;
    meta_data.angle_resolution = header.angle_resolution * 0.001;
    meta_data.contaminated_angle = header.contaminated_angle;
    meta_data.contaminated_level = header.contaminated_level;

    return meta_data;
}

void DeviceImpl::extractScanData(const std::vector<uint8_t>& scan_data,
                                 std::vector<uint16_t>& ranges,
                                 std::vector<uint16_t>& intensities) {
    int points_size = scan_data.size() / 4;
    ranges.resize(points_size);
    intensities.resize(points_size);

    for (size_t i = 0; i < scan_data.size(); i += 4) {
        uint16_t range =
            ntohs(*reinterpret_cast<const uint16_t*>(scan_data.data() + i));
        uint16_t intensity =
            ntohs(*reinterpret_cast<const uint16_t*>(scan_data.data() + i + 2));
        ranges[i / 4] = range * 2;  // Convert to millimeters
        intensities[i / 4] = intensity;
    }
}

void DeviceImpl::mergeScanData(std::vector<uint16_t>& ranges,
                               std::vector<uint16_t>& intensities) {
    size_t points_size = sum_of_scan_blocks_size_ / 4;
    ranges.resize(points_size);
    intensities.resize(points_size);
    size_t index = 0;
    while (!scan_blocks_.empty()) {
        auto& block = scan_blocks_.front();
        for (size_t i = 0; i < block.size(); i += 4) {
            uint16_t range =
                ntohs(*reinterpret_cast<const uint16_t*>(block.data() + i));
            uint16_t intensity =
                ntohs(*reinterpret_cast<const uint16_t*>(block.data() + i + 2));
            ranges[index] = range * 2;  // Convert to millimeters
            intensities[index] = intensity;
            index++;
        }
        scan_blocks_.pop();
    }
}

void DeviceImpl::clearScanDataQueue() {
    while (!scan_blocks_.empty()) {
        scan_blocks_.pop();
    }
}

void DeviceImpl::onCommandResponseReceived(const uint8_t* data, size_t size) {
    CommandResponse response;
    response.setData(data, size);
    std::unique_lock lock(command_data_queue_mutex_);
    command_data_queue_.push(response);
    command_data_queue_cv_.notify_all();
}

void DeviceImpl::setCommunicationProtocol(const LidarProtocolType& protocol) {
    // set data publish protocol, 0: UDP, 1: TCP
    if (protocol != LidarProtocolType::UNKNOWN) {
        constexpr auto protocol_option =
            LidarOption::OB_LIDAR_OPTION_TRANSFER_PROTOCOL;
        auto protocol_value = static_cast<const int>(protocol);
        setOption(protocol_option, &protocol_value, sizeof(protocol_value));
    }
}

void DeviceImpl::configureScanBlockSizeBasedOnWorkMode() {
    // get work mode
    uint32_t work_mode = 0;
    size_t size_read = 0;
    getOption(LidarOption::OB_LIDAR_OPTION_WORK_MODE, &work_mode,
              sizeof(work_mode), &size_read);
    if (work_mode == 3) {
        // I don't know why, but this is the max scan block size for 20hz
        LOG_INFO("Work mode: 3, max scan block size: 90");
        max_scan_block_size_ = 90;
        block_point_size_ = LASER_SCAN_BLOCK_LENGTH_20HZ_CALIB;
    } else {
        LOG_INFO("Work mode: {}, max scan block size: 18", work_mode);
        max_scan_block_size_ = 18;
        block_point_size_ = LASER_SCAN_BLOCK_LENGTH_20HZ;
    }
}

}  // namespace ob_lidar::detail
