#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>
namespace ob_lidar_driver {

enum class Status : int {
    OK = 0,
    ERROR = 1,
    OUT_OF_RANGE = 2,
    TIMEOUT = 3,
    INVALID = 4,
    NOT_SUPPORTED = 5,
    INTERNAL_ERROR = 6,
};

struct LidarPoint {
    int32_t x;            /**< X axis, Unit: 2mm */
    int32_t y;            /**< Y axis, Unit: 2mm */
    int32_t z;            /**< Z axis, Unit: 2mm */
    uint8_t reflectivity; /**< Reflectivity */

    /**
     * Tag:
     * bit6-7: Reserved
     * bit4-5: Detection Point Attribute -
     *     0: High Confidence (Normal Point)
     *     1: Medium Confidence
     *     2: Low Confidence
     *     3: Reserved
     * bit2-3: Detection Point Attribute - Rain, Fog, Dust, and Other Tiny
     * Particles 0: High Confidence (Normal Point) 1: Medium Confidence 2: Low
     * Confidence 3: Reserved bit0-1: Detection Point Attribute - Drag Point
     * Between Foreground and Background Objects 0: High Confidence (Normal
     * Point) 1: Medium Confidence 2: Low Confidence 3: Reserved
     */
    uint8_t tag;
};

struct LidarSpherePoint {
    uint32_t depth; /**< Distance, Unit: 2mm */
    uint16_t theta; /**< Zenith angle, Unit: 0.01 degree */
    uint16_t phi;   /**< Azimuth angle, Unit: 0.01 degree */
    uint8_t reflectivity;

    /**
     * @copydoc LidarPoint::tag
     */
    uint8_t tag;
};

/**
 * @brief Protocol type for the Lidar device.
 */
enum class LidarProtocolType : int {
    UDP = 0,
    TCP = 1,
    UNKNOWN = 0xFF,
};

/**
 * @brief Work mode for the Lidar device.
 */
enum class LidarWorkMode : int {
    MEASURE = 0,  // Measure
    STANDBY = 1,  // Standby
    SLEEP = 3,    // Sleep
    UNKNOWN = 0xFF,
};

/**
 * @brief Frame type for the Lidar device.
 */
enum class LidarFrameType : int {
    IMU = 0,
    POINT_CLOUD = 1,
    SPHERE_POINT_CLOUD = 2,
    LOG = 3,
    UNKNOWN = 0xFF,
};

/**
 * @brief Stream type for the Lidar device.
 */
enum class LidarStreamType : int {
    IMU = 0,
    POINT_CLOUD = 1,
    SPHERE_POINT_CLOUD = 2,
    LOG = 3,
    UNKNOWN = 0xFF,
};

/**
 * @brief Channel type for the Lidar device.
 */
enum class LidarChannelType : int {
    COMMAND,             // command channel
    POINT_CLOUD,         // point cloud channel
    SPHERE_POINT_CLOUD,  // sphere point cloud channel
    IMU,                 // imu channel
    LOG,                 // log channel
    UNKNOWN,
};

/**
 * @brief Status flags for the Lidar device.
 */
enum class LidarStatusFlags : int {
    // 0-1 bits: MCU temperature
    // 00: Temperature normal
    // 01: Temperature high
    // 10: Temperature low
    // 11: Reserved
    MCU_TEMPERATURE_NORMAL = 0x00000000,
    MCU_TEMPERATURE_HIGH = 0x00000001,
    MCU_TEMPERATURE_LOW = 0x00000002,
    MCU_TEMPERATURE_RESERVED = 0x00000003,

    // 2-3 bits: FPGA temperature
    // 00: Temperature normal
    // 01: Temperature high
    // 10: Temperature low
    FPGA_TEMPERATURE_NORMAL = 0x00000000 << 2,
    FPGA_TEMPERATURE_HIGH = 0x00000001 << 2,
    FPGA_TEMPERATURE_LOW = 0x00000002 << 2,

    // 3-4 bits: APD bias voltage
    // 00: Voltage normal
    // 01: Voltage high
    // 10: Voltage low
    APD_BIAS_VOLTAGE_NORMAL = 0x00000000 << 3,
    APD_BIAS_VOLTAGE_HIGH = 0x00000001 << 3,
    APD_BIAS_VOLTAGE_LOW = 0x00000002 << 3,

    // 5-6 bits: Motor status
    // 00: Normal
    // 01: Motor speed abnormal
    // 10: Motor blocked
    MOTOR_STATUS_NORMAL = 0x00000000 << 5,
    MOTOR_STATUS_ABNORMAL = 0x00000001 << 5,
    MOTOR_STATUS_BLOCKED = 0x00000002 << 5,

    // 7-8 bits: Dirty detection
    // 00: No dirt
    // 01: Cover dirty
    DIRTY_DETECTION_NONE = 0x00000000 << 7,
    DIRTY_DETECTION_COVER = 0x00000001 << 7,

    // 9th bit: Network detection
    // 0: Normal
    // 1: Network disconnected
    NETWORK_DETECTION_NORMAL = 0x00000000 << 9,
    NETWORK_DETECTION_DISCONNECTED = 0x00000001 << 9,

    // 10-31 bits: Reserved
    RESERVED = 0x00000000 << 10,
};

/**
 * @brief Echo mode for the Lidar device.
 */
enum class LidarEchoMode : int {
    FIRST = 0,  // first echo
    LAST = 1,   // last echo
    UNKNOWN = 0xFF,
};

struct DeviceInfo {
    std::string device_name;
    std::string model;
    std::string serial_number;
    std::string firmware_version;
    std::string fpga_version;
};

/**
 * @brief Product model type for the Lidar device.
 */
enum class LidarModel {
    MS600,
    TL2401,
    UNKNOWN,
};

/**
 * @brief Log level for the Lidar device.
 */
enum LogLevel {
    INFO,
    WARNING,
    ERROR,
    TRACE,
    DEBUG,
    CRITICAL,
};

}  // namespace ob_lidar_driver
