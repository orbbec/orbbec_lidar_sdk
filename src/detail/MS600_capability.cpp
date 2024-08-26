#include "MS600_capability.hpp"
#include "../logger.hpp"
#include <magic_enum/magic_enum.hpp>

namespace ob_lidar {

MS600LidarCapabilities::MS600LidarCapabilities()
    : LidarCapabilitiesInterface() {
    // IP address, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_IP_ADDRESS, READ_WRITE);
    // port, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_PORT, READ_WRITE);
    // mac address, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_MAC_ADDRESS, READ_WRITE);
    // subnet mask, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_SUBNET_MASK, READ_WRITE);
    // scan speed, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_SCAN_SPEED, READ_WRITE);
    // scan direction, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_SCAN_DIRECTION, READ_WRITE);
    // transfer protocol, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_TRANSFER_PROTOCOL, READ_WRITE);
    // work mode, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_WORK_MODE, READ_WRITE);
    // initiate device connection, write only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_INITIATE_DEVICE_CONNECTION,
                               WRITE_ONLY);
    // serial number, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_SERIAL_NUMBER, READ_WRITE);
    // reboot, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_REBOOT, READ_WRITE);
    // factory mode, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_FACTORY_MODE, READ_WRITE);
    // echo mode, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_ECHO_MODE, READ_WRITE);
    // apply configs, write only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_APPLY_CONFIGS, WRITE_ONLY);
    // enable streaming, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_ENABLE_STREAMING, READ_WRITE);
    // filter level, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_FILTER_LEVEL, READ_WRITE);
    // product model, read only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_PRODUCT_MODEL, READ_ONLY);
    // firmware version, read only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_FIRMWARE_VERSION, READ_ONLY);
    // FPGA version, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_FPGA_VERSION, READ_WRITE);
    // spin speed, read only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_SPIN_SPEED, READ_ONLY);
    // MCU temperature, read only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_MCU_TEMPERATURE, READ_ONLY);
    // FPGA temperature, read only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_FPGA_TEMPERATURE, READ_ONLY);
    // FPGA version date, read only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_FPGA_VERSION_DATE, READ_ONLY);
    // high voltage, read only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_HIGH_VOLTAGE, READ_ONLY);
    // APD temperature, read only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_APD_TEMPERATURE, READ_ONLY);
    // TX voltage, read only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_TX_VOLTAGE, READ_ONLY);
    // filter level, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_FILTER_LEVEL, READ_WRITE);
    // imu, not supported
    REGISTER_STREAM(LidarStreamType::IMU, false);
    // point cloud, not supported
    REGISTER_STREAM(LidarStreamType::POINT_CLOUD, false);
    // sphere point cloud, not supported
    REGISTER_STREAM(LidarStreamType::SPHERE_POINT_CLOUD, false);
    // scan data, supported
    REGISTER_STREAM(LidarStreamType::SCAN, true);
    // ms600 supports 15hz, 20hz, 25hz, 30hz
    supported_stream_freq_map_[LidarStreamType::SCAN] = {15.0, 20.0, 25.0,
                                                         30.0};
}

bool MS600LidarCapabilities::isOptionSupported(
    const LidarOption &option) const {
    return supported_options_map_.find(option) != supported_options_map_.end();
}

bool MS600LidarCapabilities::isOptionSupported(
    const LidarOption &type, const LidarOptionPermission &permission) const {
    return supported_options_map_.find(type) != supported_options_map_.end() &&
           supported_options_map_.at(type) == permission;
}

bool MS600LidarCapabilities::isStreamSupported(
    const LidarStreamType &type) const {
    return supported_stream_map_.find(type) != supported_stream_map_.end() &&
           supported_stream_map_.at(type);
}

bool MS600LidarCapabilities::isStreamFreqSupported(const LidarStreamType &type,
                                                   const double &freq) const {
    if (!isStreamSupported(type)) {
        LOG_ERROR("Stream type {} is not supported", magic_enum::enum_name(type));
        return false;
    }
    auto freq_list = supported_stream_freq_map_.at(type);
    return std::find(freq_list.begin(), freq_list.end(), freq) !=
           freq_list.end();
}

}  // namespace ob_lidar
