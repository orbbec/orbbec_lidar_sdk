#pragma once
#include <cstdint>
#include <map>

#include "orb_lidar_driver/types.hpp"

namespace ob_lidar_driver {

enum  LidarCommandInterface  {
    // set lidar ip address
    SET_IP_ADDR = 0x0101,
    // set lidar port
    SET_PORT = 0x0102,
    // set lidar mac address
    SET_MAC_ADDR = 0x0103,
    //  set lidar subnet mask
    SET_SUBNET_MASK = 0x0104,
    // set lidar scan speed
    SET_SCAN_SPEED = 0x0105,
    // set lidar scan direction
    SET_SCAN_DIRECTION = 0x0106,
    // set lidar transfer protocol
    SET_TRANSFER_PROTOCOL = 0x0107,
    // set lidar work mode
    SET_WORK_MODE = 0x0108,
    // set lidar initiate device connection
    SET_INITIATE_DEVICE_CONNECTION = 0x0109,
    // set lidar serial number
    SET_SERIAL_NUMBER = 0x010A,
    // reboot device
    REBOOT_DEVICE = 0x010B,
    // enter factory mode
    ENTER_FACTORY_MODE = 0x010C,
    // set lidar echo mode
    SET_ECHO_MODE = 0x010D,

    // apply configs
    APPLY_CONFIGS = 0x010E,
    // enable or disable streaming
    ENABLE_STREAMING = 0x010F,
    // set filter level
    SET_FILTER_LEVEL = 0x0111,
    // start mcu upgrade
    START_MCU_UPGRADE = 0x01AB,
    // end mcu upgrade
    END_MCU_UPGRADE = 0x01AC,
    // project id verification
    PROJECT_ID_VERIFICATION = 0x01AD,
    // product id verification
    PRODUCT_ID_VERIFICATION = 0x01AE,
    // send md5 value
    SEND_MD5_VALUE = 0x01AF,
    //  verify md5 value
    VERIFY_MD5_VALUE = 0x01B0,
    // transfer firmware upgrade package
    TRANSFER_FIRMWARE_UPGRADE_PACKAGE = 0x01B1,
    // start fpga upgrade
    START_FPGA_UPGRADE = 0x01B2,
    // end fpga upgrade
    END_FPGA_UPGRADE = 0x01B3,
    // transfer fpga upgrade package
    TRANSFER_FPGA_UPGRADE_PACKAGE = 0x01B4,
    // start MEMS upgrade
    START_MEMS_UPGRADE = 0x01C0,
    // end MEMS upgrade
    END_MEMS_UPGRADE = 0x01C1,
    // MEMS id verification
    MEMS_ID_VERIFICATION = 0x01C2,
    // send MEMS md5 value
    SEND_MEMS_MD5_VALUE = 0x01C3,
    // verify MEMS md5 value
    VERIFY_MEMS_MD5_VALUE = 0x01C4,
    // transfer MEMS upgrade package
    TRANSFER_MEMS_UPGRADE_PACKAGE = 0x01C5,
    // get ip address
    GET_IP_ADDR = 0x0201,
    // get port
    GET_PORT = 0x0202,
    // get mac address
    GET_MAC_ADDR = 0x0203,
    // get subnet mask
    GET_SUBNET_MASK = 0x0204,
    // get scan speed
    GET_SCAN_SPEED = 0x0205,
    // get scan direction
    GET_SCAN_DIRECTION = 0x0206,
    // get transfer protocol
    GET_TRANSFER_PROTOCOL = 0x0207,
    // get work mode
    GET_WORK_MODE = 0x0208,
    // get device serial number
    GET_SERIAL_NUMBER = 0x0209,
    // get product model
    GET_PRODUCT_MODEL = 0x020A,
    // get firmware version
    GET_FIRMWARE_VERSION = 0x020B,
    // get FPGA version
    GET_FPGA_VERSION = 0x020C,
    // get status info
    GET_STATUS_INFO = 0x020D,
    // get warning info
    GET_WARNING_INFO = 0x020E,
    // get echo mode
    GET_ECHO_MODE = 0x020F,
    // get spin speed
    GET_SPIN_SPEED = 0x0210,
    // get MCU temperature
    GET_MCU_TEMPERATURE = 0x0211,
    // get FPGA temperature
    GET_FPGA_TEMPERATURE = 0x0212,
    // get FPGA version date
    GET_FPGA_VERSION_DATE = 0x0213,
    // get high voltage
    GET_HIGH_VOLTAGE = 0x0214,
    // get APD temperature
    GET_APD_TEMPERATURE = 0x0215,
    // get TX voltage
    GET_TX_VOLTAGE = 0x0216,
    // get filter level
    GET_FILTER_LEVEL = 0x0218,
    // unknown command
    UNKNOWN = 0xFFFF,
};

}  // namespace ob_lidar_driver
