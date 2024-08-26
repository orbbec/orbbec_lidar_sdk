#pragma once
#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "types.hpp"

namespace ob_lidar_driver {

/**
 * @brief Enumeration of various Lidar options. you can also call it as
 * Command for the Lidar device.
 */

enum LidarOption {
    // IP address
    OB_LIDAR_OPTION_IP_ADDRESS,
    // Port
    OB_LIDAR_OPTION_PORT,
    // MAC address
    OB_LIDAR_OPTION_MAC_ADDRESS,
    // Subnet mask
    OB_LIDAR_OPTION_SUBNET_MASK,
    // Scan speed
    OB_LIDAR_OPTION_SCAN_SPEED,
    // Scan direction
    OB_LIDAR_OPTION_SCAN_DIRECTION,
    // Transfer Protocol (TCP/UDP)
    OB_LIDAR_OPTION_TRANSFER_PROTOCOL,
    // work mode
    OB_LIDAR_OPTION_WORK_MODE,
    // Initiate device connection
    OB_LIDAR_OPTION_INITIATE_DEVICE_CONNECTION,
    //  serial number
    OB_LIDAR_OPTION_SERIAL_NUMBER,
    // reboot device
    OB_LIDAR_OPTION_REBOOT,
    // enter factory mode
    OB_LIDAR_OPTION_FACTORY_MODE,
    // echo mode
    OB_LIDAR_OPTION_ECHO_MODE,
    // apply configs
    OB_LIDAR_OPTION_APPLY_CONFIGS,
    // enable or disable streaming
    OB_LIDAR_OPTION_ENABLE_STREAMING,
    // filter level
    OB_LIDAR_OPTION_FILTER_LEVEL,
    // start mcu upgrade
    OB_LIDAR_OPTION_START_MCU_UPGRADE,
    // end mcu upgrade
    OB_LIDAR_OPTION_END_MCU_UPGRADE,
    // Project ID verification
    OB_LIDAR_OPTION_PROJECT_ID_VERIFICATION,
    // Product ID verification
    OB_LIDAR_OPTION_PRODUCT_ID_VERIFICATION,
    // send md5 value
    OB_LIDAR_OPTION_SEND_MD5_VALUE,
    // verify md5 value
    OB_LIDAR_OPTION_VERIFY_MD5_VALUE,
    // transfer firmware upgrade package
    OB_LIDAR_OPTION_TRANSFER_FIRMWARE_UPGRADE_PACKAGE,
    // start fpga upgrade
    OB_LIDAR_OPTION_START_FPGA_UPGRADE,
    // end fpga upgrade
    OB_LIDAR_OPTION_END_FPGA_UPGRADE,
    // transfer fpga upgrade package
    OB_LIDAR_OPTION_TRANSFER_FPGA_UPGRADE_PACKAGE,
    // start MEMS upgrade
    OB_LIDAR_OPTION_START_MEMS_UPGRADE,
    // end MEMS upgrade
    OB_LIDAR_OPTION_END_MEMS_UPGRADE,
    // MEMS ID verification
    OB_LIDAR_OPTION_MEMS_ID_VERIFICATION,
    // send MEMS md5 value
    OB_LIDAR_OPTION_SEND_MEMS_MD5_VALUE,
    // verify MEMS md5 value
    OB_LIDAR_OPTION_VERIFY_MEMS_MD5_VALUE,
    // transfer MEMS upgrade package
    OB_LIDAR_OPTION_TRANSFER_MEMS_UPGRADE_PACKAGE,
    // product model
    OB_LIDAR_OPTION_PRODUCT_MODEL,
    // firmware version
    OB_LIDAR_OPTION_FIRMWARE_VERSION,
    // FPGA version
    OB_LIDAR_OPTION_FPGA_VERSION,
    // spin speed
    OB_LIDAR_OPTION_SPIN_SPEED,
    // MCU temperature
    OB_LIDAR_OPTION_MCU_TEMPERATURE,
    // FPGA temperature
    OB_LIDAR_OPTION_FPGA_TEMPERATURE,
    // FPGA version date
    OB_LIDAR_OPTION_FPGA_VERSION_DATE,
    // high voltage
    OB_LIDAR_OPTION_HIGH_VOLTAGE,
    // APD temperature
    OB_LIDAR_OPTION_APD_TEMPERATURE,
    // TX voltage
    OB_LIDAR_OPTION_TX_VOLTAGE,

};

/**
 * @brief Permission for the Lidar option.
 */
enum LidarOptionPermission {
    // read only
    READ_ONLY = 0,
    // write only
    WRITE_ONLY = 1,
    // read and write
    READ_WRITE = 2
};

/**
 * @brief Operation for the Lidar option.
 */
enum class LidarOptionOp {
    GET,  // get the value of the option
    SET,  // set the value of the option
};

template <LidarOption>
struct LidarOptionTrait;

#define DEFINE_LIDAR_OPTION_TRAIT(option, option_type) \
    template <>                                        \
    struct LidarOptionTrait<option> {                  \
        using type = option_type;                      \
    };

// define the type of each option
// FIXME: some options type is not clear
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_IP_ADDRESS, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_PORT, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_MAC_ADDRESS, std::string)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_SUBNET_MASK, std::string)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_SCAN_SPEED, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_SCAN_DIRECTION, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_TRANSFER_PROTOCOL, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_WORK_MODE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_INITIATE_DEVICE_CONNECTION, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_SERIAL_NUMBER, std::string)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_REBOOT, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_FACTORY_MODE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_ECHO_MODE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_APPLY_CONFIGS, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_ENABLE_STREAMING, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_FILTER_LEVEL, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_START_MCU_UPGRADE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_END_MCU_UPGRADE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_PROJECT_ID_VERIFICATION, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_PRODUCT_ID_VERIFICATION, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_SEND_MD5_VALUE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_VERIFY_MD5_VALUE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_TRANSFER_FIRMWARE_UPGRADE_PACKAGE,
                          int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_START_FPGA_UPGRADE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_END_FPGA_UPGRADE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_TRANSFER_FPGA_UPGRADE_PACKAGE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_START_MEMS_UPGRADE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_END_MEMS_UPGRADE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_MEMS_ID_VERIFICATION, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_SEND_MEMS_MD5_VALUE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_VERIFY_MEMS_MD5_VALUE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_TRANSFER_MEMS_UPGRADE_PACKAGE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_PRODUCT_MODEL, std::string)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_FIRMWARE_VERSION, std::string)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_FPGA_VERSION, std::string)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_FPGA_VERSION_DATE, std::string)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_HIGH_VOLTAGE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_APD_TEMPERATURE, int)
DEFINE_LIDAR_OPTION_TRAIT(OB_LIDAR_OPTION_TX_VOLTAGE, int)

// Option size trait
template <LidarOption>
struct LidarOptionSizeTrait;

#define DEFINE_LIDAR_OPTION_SIZE_TRAIT(option, option_size) \
    template <>                                             \
    struct LidarOptionSizeTrait<option> {                   \
        static constexpr size_t size = option_size;         \
    };

// serial number 16 bytes
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_SERIAL_NUMBER, 16)
// firmware version 16 bytes
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_FIRMWARE_VERSION, 16)
// fpga version 16 bytes
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_FPGA_VERSION, 16)
// mac address 8 bytes,use first 6 bytes
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_MAC_ADDRESS, 8)

DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_PRODUCT_MODEL, 16)

DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_IP_ADDRESS, sizeof(uint32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_PORT, sizeof(uint32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_SUBNET_MASK, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_SCAN_SPEED, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_SCAN_DIRECTION, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_TRANSFER_PROTOCOL,
                               sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_WORK_MODE, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_INITIATE_DEVICE_CONNECTION,
                               sizeof(int32_t))

DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_REBOOT, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_FACTORY_MODE, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_ECHO_MODE, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_APPLY_CONFIGS, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_ENABLE_STREAMING,
                               sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_FILTER_LEVEL, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_START_MCU_UPGRADE,
                               sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_END_MCU_UPGRADE, sizeof(int32_t))
// FIXME: the size of the following options size is not clear
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_PROJECT_ID_VERIFICATION,
                               sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_PRODUCT_ID_VERIFICATION,
                               sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_SEND_MD5_VALUE, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_VERIFY_MD5_VALUE,
                               sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(
    OB_LIDAR_OPTION_TRANSFER_FIRMWARE_UPGRADE_PACKAGE, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_START_FPGA_UPGRADE,
                               sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_END_FPGA_UPGRADE,
                               sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_TRANSFER_FPGA_UPGRADE_PACKAGE,
                               sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_START_MEMS_UPGRADE,
                               sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_END_MEMS_UPGRADE,
                               sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_MEMS_ID_VERIFICATION,
                               sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_SEND_MEMS_MD5_VALUE,
                               sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_VERIFY_MEMS_MD5_VALUE,
                               sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_TRANSFER_MEMS_UPGRADE_PACKAGE,
                               sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_HIGH_VOLTAGE, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_APD_TEMPERATURE, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_TX_VOLTAGE, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_SPIN_SPEED, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_MCU_TEMPERATURE, sizeof(int32_t))
DEFINE_LIDAR_OPTION_SIZE_TRAIT(OB_LIDAR_OPTION_FPGA_TEMPERATURE,
                               sizeof(int32_t))

}  // namespace ob_lidar_driver