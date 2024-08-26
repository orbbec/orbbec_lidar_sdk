#include "MS600_capability.hpp"

namespace ob_lidar_driver {

MS600LidarCapabilities::MS600LidarCapabilities()
    : LidarCapabilitiesInterface() {
    // hard code the supported options and streams
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_IP_ADDRESS, READ_WRITE);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_PORT, READ_WRITE);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_MAC_ADDRESS, READ_WRITE);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_SUBNET_MASK, READ_WRITE);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_SCAN_SPEED, READ_WRITE);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_SCAN_DIRECTION, READ_WRITE);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_TRANSFER_PROTOCOL, READ_WRITE);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_WORK_MODE, READ_WRITE);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_INITIATE_DEVICE_CONNECTION,
                               WRITE_ONLY);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_SERIAL_NUMBER, READ_WRITE);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_REBOOT, READ_WRITE);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_FACTORY_MODE, READ_WRITE);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_ECHO_MODE, READ_WRITE);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_APPLY_CONFIGS, READ_WRITE);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_ENABLE_STREAMING, READ_WRITE);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_FILTER_LEVEL, READ_WRITE);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_START_MCU_UPGRADE, READ_WRITE);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_END_MCU_UPGRADE, WRITE_ONLY);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_PROJECT_ID_VERIFICATION,
                               WRITE_ONLY);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_PRODUCT_ID_VERIFICATION,
                               WRITE_ONLY);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_SEND_MD5_VALUE, WRITE_ONLY);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_VERIFY_MD5_VALUE, WRITE_ONLY);
    REGISTER_OPTION_PERMISSION(
        OB_LIDAR_OPTION_TRANSFER_FIRMWARE_UPGRADE_PACKAGE, WRITE_ONLY);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_START_FPGA_UPGRADE, WRITE_ONLY);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_END_FPGA_UPGRADE, WRITE_ONLY);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_TRANSFER_FPGA_UPGRADE_PACKAGE,
                               WRITE_ONLY);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_START_MEMS_UPGRADE, WRITE_ONLY);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_END_MEMS_UPGRADE, WRITE_ONLY);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_MEMS_ID_VERIFICATION,
                               WRITE_ONLY);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_SEND_MEMS_MD5_VALUE, WRITE_ONLY);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_VERIFY_MEMS_MD5_VALUE,
                               WRITE_ONLY);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_TRANSFER_MEMS_UPGRADE_PACKAGE,
                               WRITE_ONLY);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_PRODUCT_MODEL, READ_ONLY);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_FIRMWARE_VERSION, READ_ONLY);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_FPGA_VERSION, READ_WRITE);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_SPIN_SPEED, READ_ONLY);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_MCU_TEMPERATURE, READ_ONLY);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_FPGA_TEMPERATURE, READ_ONLY);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_FPGA_VERSION_DATE, READ_ONLY);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_HIGH_VOLTAGE, READ_ONLY);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_APD_TEMPERATURE, READ_ONLY);
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_TX_VOLTAGE, READ_ONLY);

    REGISTER_STREAM(LidarStreamType::IMU, false);
    REGISTER_STREAM(LidarStreamType::POINT_CLOUD, true);
    REGISTER_STREAM(LidarStreamType::SPHERE_POINT_CLOUD, true);
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
    return false;
}

}  // namespace ob_lidar_driver