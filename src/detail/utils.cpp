#include "./utils.hpp"
namespace ob_lidar {
std::string commandIDToString(uint16_t command_id) {
    switch (command_id) {
        case SET_IP_ADDR:
            return "SET_IP_ADDR";
        case SET_PORT:
            return "SET_PORT";
        case SET_MAC_ADDR:
            return "SET_MAC_ADDR";
        case SET_SUBNET_MASK:
            return "SET_SUBNET_MASK";
        case SET_SCAN_SPEED:
            return "SET_SCAN_SPEED";
        case SET_SCAN_DIRECTION:
            return "SET_SCAN_DIRECTION";
        case SET_TRANSFER_PROTOCOL:
            return "SET_TRANSFER_PROTOCOL";
        case SET_WORK_MODE:
            return "SET_WORK_MODE";
        case SET_INITIATE_DEVICE_CONNECTION:
            return "SET_INITIATE_DEVICE_CONNECTION";
        case SET_SERIAL_NUMBER:
            return "SET_SERIAL_NUMBER";
        case REBOOT_DEVICE:
            return "REBOOT_DEVICE";
        case ENTER_FACTORY_MODE:
            return "ENTER_FACTORY_MODE";
        case SET_ECHO_MODE:
            return "SET_ECHO_MODE";
        case APPLY_CONFIGS:
            return "APPLY_CONFIGS";
        case ENABLE_STREAMING:
            return "ENABLE_STREAMING";
        case SET_FILTER_LEVEL:
            return "SET_FILTER_LEVEL";
        case START_MCU_UPGRADE:
            return "START_MCU_UPGRADE";
        case END_MCU_UPGRADE:
            return "END_MCU_UPGRADE";
        case PROJECT_ID_VERIFICATION:
            return "PROJECT_ID_VERIFICATION";
        case PRODUCT_ID_VERIFICATION:
            return "PRODUCT_ID_VERIFICATION";
        case SEND_MD5_VALUE:
            return "SEND_MD5_VALUE";
        case VERIFY_MD5_VALUE:
            return "VERIFY_MD5_VALUE";
        case TRANSFER_FIRMWARE_UPGRADE_PACKAGE:
            return "TRANSFER_FIRMWARE_UPGRADE_PACKAGE";
        case START_FPGA_UPGRADE:
            return "START_FPGA_UPGRADE";
        case END_FPGA_UPGRADE:
            return "END_FPGA_UPGRADE";
        case TRANSFER_FPGA_UPGRADE_PACKAGE:
            return "TRANSFER_FPGA_UPGRADE_PACKAGE";
        case START_MEMS_UPGRADE:
            return "START_MEMS_UPGRADE";
        case END_MEMS_UPGRADE:
            return "END_MEMS_UPGRADE";
        case MEMS_ID_VERIFICATION:
            return "MEMS_ID_VERIFICATION";
        case SEND_MEMS_MD5_VALUE:
            return "SEND_MEMS_MD5_VALUE";
        case VERIFY_MEMS_MD5_VALUE:
            return "VERIFY_MEMS_MD5_VALUE";
        case TRANSFER_MEMS_UPGRADE_PACKAGE:
            return "TRANSFER_MEMS_UPGRADE_PACKAGE";
        case GET_IP_ADDR:
            return "GET_IP_ADDR";
        case GET_PORT:
            return "GET_PORT";
        case GET_MAC_ADDR:
            return "GET_MAC_ADDR";
        case GET_SUBNET_MASK:
            return "GET_SUBNET_MASK";
        case GET_SCAN_SPEED:
            return "GET_SCAN_SPEED";
        case GET_SCAN_DIRECTION:
            return "GET_SCAN_DIRECTION";
        case GET_TRANSFER_PROTOCOL:
            return "GET_TRANSFER_PROTOCOL";
        case GET_WORK_MODE:
            return "GET_WORK_MODE";
        case GET_SERIAL_NUMBER:
            return "GET_SERIAL_NUMBER";
        case GET_PRODUCT_MODEL:
            return "GET_PRODUCT_MODEL";
        case GET_FIRMWARE_VERSION:
            return "GET_FIRMWARE_VERSION";
        case GET_FPGA_VERSION:
            return "GET_FPGA_VERSION";
        case GET_STATUS_INFO:
            return "GET_STATUS_INFO";
        case GET_WARNING_INFO:
            return "GET_WARNING_INFO";
        case GET_ECHO_MODE:
            return "GET_ECHO_MODE";
        case GET_SPIN_SPEED:
            return "GET_SPIN_SPEED";
        case GET_MCU_TEMPERATURE:
            return "GET_MCU_TEMPERATURE";
        case GET_FPGA_TEMPERATURE:
            return "GET_FPGA_TEMPERATURE";
        case GET_FPGA_VERSION_DATE:
            return "GET_FPGA_VERSION_DATE";
        case GET_HIGH_VOLTAGE:
            return "GET_HIGH_VOLTAGE";
        case GET_APD_TEMPERATURE:
            return "GET_APD_TEMPERATURE";
        case GET_TX_VOLTAGE:
            return "GET_TX_VOLTAGE";
        case GET_FILTER_LEVEL:
            return "GET_FILTER_LEVEL";
        default:
            return "UNKNOWN";
    }
}
uint8_t calcCrc8(const uint8_t *p, uint8_t len) {
    static const uint8_t crc8_table[256] = {
        0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
        0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
        0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
        0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
        0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
        0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
        0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
        0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
        0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
        0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
        0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
        0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
        0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
        0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
        0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
        0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
        0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
        0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
        0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
        0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
        0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
        0x7f, 0x32, 0xe5, 0xa8};

    uint8_t crc = 0;
    uint16_t i;
    for (i = 0; i < len; i++) {
        crc = crc8_table[(crc ^ *p++) & 0xff];
    }
    return crc;
}

}  // namespace ob_lidar
