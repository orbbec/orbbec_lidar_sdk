#include "TL2401_capability.hpp"

namespace ob_lidar {

TL2401LidarCapabilities::TL2401LidarCapabilities() {
    // hard code the supported options and streams
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
    // apply configs, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_APPLY_CONFIGS, READ_WRITE);
    // enable streaming, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_ENABLE_STREAMING, READ_WRITE);
    // filter level, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_FILTER_LEVEL, READ_WRITE);
    // start mcu upgrade, read/write
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_START_MCU_UPGRADE, READ_WRITE);
    // end mcu upgrade, write only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_END_MCU_UPGRADE, WRITE_ONLY);
    // project id verification, write only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_PROJECT_ID_VERIFICATION,
                               WRITE_ONLY);
    // product id verification, write only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_PRODUCT_ID_VERIFICATION,
                               WRITE_ONLY);
    // send md5 value, write only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_SEND_MD5_VALUE, WRITE_ONLY);
    // verify md5 value, write only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_VERIFY_MD5_VALUE, WRITE_ONLY);
    // transfer firmware upgrade package, write only
    REGISTER_OPTION_PERMISSION(
        OB_LIDAR_OPTION_TRANSFER_FIRMWARE_UPGRADE_PACKAGE, WRITE_ONLY);
    // start fpga upgrade, write only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_START_FPGA_UPGRADE, WRITE_ONLY);
    // end fpga upgrade, write only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_END_FPGA_UPGRADE, WRITE_ONLY);
    // transfer fpga upgrade package, write only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_TRANSFER_FPGA_UPGRADE_PACKAGE,
                               WRITE_ONLY);
    // start mems upgrade, write only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_START_MEMS_UPGRADE, WRITE_ONLY);
    // end mems upgrade, write only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_END_MEMS_UPGRADE, WRITE_ONLY);
    // MEMS id verification, write only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_MEMS_ID_VERIFICATION,
                               WRITE_ONLY);
    // send MEMS md5 value, write only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_SEND_MEMS_MD5_VALUE, WRITE_ONLY);
    // verify MEMS md5 value, write only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_VERIFY_MEMS_MD5_VALUE,
                               WRITE_ONLY);
    // transfer MEMS upgrade package, write only
    REGISTER_OPTION_PERMISSION(OB_LIDAR_OPTION_TRANSFER_MEMS_UPGRADE_PACKAGE,
                               WRITE_ONLY);
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
    // imu, not supported
    REGISTER_STREAM(LidarStreamType::IMU, true);
    // point cloud, not supported
    REGISTER_STREAM(LidarStreamType::POINT_CLOUD, true);
    // sphere point cloud, not supported
    REGISTER_STREAM(LidarStreamType::SPHERE_POINT_CLOUD, true);
    // scan data, not supported
    REGISTER_STREAM(LidarStreamType::SCAN, false);
}

bool TL2401LidarCapabilities::isOptionSupported(
    const LidarOption &option) const {
    return supported_options_map_.find(option) != supported_options_map_.end();
}

bool TL2401LidarCapabilities::isOptionSupported(
    const LidarOption &type, const LidarOptionPermission &permission) const {
    return supported_options_map_.find(type) != supported_options_map_.end() &&
           supported_options_map_.at(type) == permission;
}

bool TL2401LidarCapabilities::isStreamSupported(
    const LidarStreamType &type) const {
    return supported_stream_map_.find(type) != supported_stream_map_.end() &&
           supported_stream_map_.at(type);
}

bool TL2401LidarCapabilities::isStreamFreqSupported(const LidarStreamType &type,
                                                    const double &freq) const {
    return false;
}

}  // namespace ob_lidar
