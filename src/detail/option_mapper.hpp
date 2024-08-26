#pragma once
namespace ob_lidar {

class LidarOptionMapper {
   public:
    using LidarCommandInterfacePair =
        std::pair<LidarCommandInterface, LidarCommandInterface>;

    static LidarCommandInterface getCommand(LidarOption option, bool is_set) {
        const auto &commandMap = getCommandMap();
        auto it = commandMap.find(option);
        if (it != commandMap.end()) {
            return is_set ? it->second.first : it->second.second;
        }
        return LidarCommandInterface::UNKNOWN;
    }

    static LidarCommandInterface getSetCommandInterface(LidarOption option) {
        return getCommand(option, true);
    }

    static LidarCommandInterface getGetCommandInterface(LidarOption option) {
        return getCommand(option, false);
    }

   private:
    static const std::unordered_map<LidarOption, LidarCommandInterfacePair>
        &getCommandMap() {
        static const std::unordered_map<LidarOption, LidarCommandInterfacePair>
            commandMap = initializeCommandMap();
        return commandMap;
    }

    static std::unordered_map<LidarOption, LidarCommandInterfacePair>
    initializeCommandMap() {
        std::unordered_map<LidarOption, LidarCommandInterfacePair> map;
        // ip address, read/write
        map.emplace(OB_LIDAR_OPTION_IP_ADDRESS,
                    std::make_pair(LidarCommandInterface::SET_IP_ADDR,
                                   LidarCommandInterface::GET_IP_ADDR));
        // port, read/write
        map.emplace(OB_LIDAR_OPTION_PORT,
                    std::make_pair(LidarCommandInterface::SET_PORT,
                                   LidarCommandInterface::GET_PORT));
        // mac address, read/write
        map.emplace(OB_LIDAR_OPTION_MAC_ADDRESS,
                    std::make_pair(LidarCommandInterface::SET_MAC_ADDR,
                                   LidarCommandInterface::GET_MAC_ADDR));
        // subnet mask, read/write
        map.emplace(OB_LIDAR_OPTION_SUBNET_MASK,
                    std::make_pair(LidarCommandInterface::SET_SUBNET_MASK,
                                   LidarCommandInterface::GET_SUBNET_MASK));
        // scan speed
        map.emplace(OB_LIDAR_OPTION_SCAN_SPEED,
                    std::make_pair(LidarCommandInterface::SET_SCAN_SPEED,
                                   LidarCommandInterface::GET_SCAN_SPEED));
        // scan direction, read/write
        map.emplace(OB_LIDAR_OPTION_SCAN_DIRECTION,
                    std::make_pair(LidarCommandInterface::SET_SCAN_DIRECTION,
                                   LidarCommandInterface::GET_SCAN_DIRECTION));
        // transfer protocol(UDP/TCP), read/write
        map.emplace(
            OB_LIDAR_OPTION_TRANSFER_PROTOCOL,
            std::make_pair(LidarCommandInterface::SET_TRANSFER_PROTOCOL,
                           LidarCommandInterface::GET_TRANSFER_PROTOCOL));
        // work mode, read/write
        map.emplace(OB_LIDAR_OPTION_WORK_MODE,
                    std::make_pair(LidarCommandInterface::SET_WORK_MODE,
                                   LidarCommandInterface::GET_WORK_MODE));
        // Initiate device connection, write only
        map.emplace(OB_LIDAR_OPTION_INITIATE_DEVICE_CONNECTION,
                    std::make_pair(
                        LidarCommandInterface::SET_INITIATE_DEVICE_CONNECTION,
                        LidarCommandInterface::UNKNOWN));
        // serial number, read/write
        map.emplace(OB_LIDAR_OPTION_SERIAL_NUMBER,
                    std::make_pair(LidarCommandInterface::SET_SERIAL_NUMBER,
                                   LidarCommandInterface::GET_SERIAL_NUMBER));
        // reboot device, write only
        map.emplace(OB_LIDAR_OPTION_REBOOT,
                    std::make_pair(LidarCommandInterface::REBOOT_DEVICE,
                                   LidarCommandInterface::UNKNOWN));
        // enter factory mode, write only
        map.emplace(OB_LIDAR_OPTION_FACTORY_MODE,
                    std::make_pair(LidarCommandInterface::ENTER_FACTORY_MODE,
                                   LidarCommandInterface::UNKNOWN));
        // echo mode, read/write
        map.emplace(OB_LIDAR_OPTION_ECHO_MODE,
                    std::make_pair(LidarCommandInterface::SET_ECHO_MODE,
                                   LidarCommandInterface::GET_ECHO_MODE));

        // apply configs, write only
        map.emplace(OB_LIDAR_OPTION_APPLY_CONFIGS,
                    std::make_pair(LidarCommandInterface::APPLY_CONFIGS,
                                   LidarCommandInterface::UNKNOWN));

        // enable or disable streaming, write only
        map.emplace(OB_LIDAR_OPTION_ENABLE_STREAMING,
                    std::make_pair(LidarCommandInterface::ENABLE_STREAMING,
                                   LidarCommandInterface::UNKNOWN));
        // filter level, read/write
        map.emplace(OB_LIDAR_OPTION_FILTER_LEVEL,
                    std::make_pair(LidarCommandInterface::SET_FILTER_LEVEL,
                                   LidarCommandInterface::GET_FILTER_LEVEL));

        // start mcu upgrade
        map.emplace(OB_LIDAR_OPTION_START_MCU_UPGRADE,
                    std::make_pair(LidarCommandInterface::START_MCU_UPGRADE,
                                   LidarCommandInterface::UNKNOWN));
        // end mcu upgrade, write only
        map.emplace(OB_LIDAR_OPTION_END_MCU_UPGRADE,
                    std::make_pair(LidarCommandInterface::END_MCU_UPGRADE,
                                   LidarCommandInterface::UNKNOWN));

        // Project ID verification, write only
        map.emplace(
            OB_LIDAR_OPTION_PROJECT_ID_VERIFICATION,
            std::make_pair(LidarCommandInterface::PROJECT_ID_VERIFICATION,
                           LidarCommandInterface::UNKNOWN));

        // Product ID verification, write only
        map.emplace(
            OB_LIDAR_OPTION_PRODUCT_ID_VERIFICATION,
            std::make_pair(LidarCommandInterface::PRODUCT_ID_VERIFICATION,
                           LidarCommandInterface::UNKNOWN));
        // send md5 value, write only
        map.emplace(OB_LIDAR_OPTION_SEND_MD5_VALUE,
                    std::make_pair(LidarCommandInterface::SEND_MD5_VALUE,
                                   LidarCommandInterface::UNKNOWN));
        // verify md5 value, write only
        map.emplace(OB_LIDAR_OPTION_VERIFY_MD5_VALUE,
                    std::make_pair(LidarCommandInterface::VERIFY_MD5_VALUE,
                                   LidarCommandInterface::UNKNOWN));
        // transfer firmware upgrade package, write only
        map.emplace(
            OB_LIDAR_OPTION_TRANSFER_FIRMWARE_UPGRADE_PACKAGE,
            std::make_pair(
                LidarCommandInterface::TRANSFER_FIRMWARE_UPGRADE_PACKAGE,
                LidarCommandInterface::UNKNOWN));
        // start fpga upgrade, write only
        map.emplace(OB_LIDAR_OPTION_START_FPGA_UPGRADE,
                    std::make_pair(LidarCommandInterface::START_FPGA_UPGRADE,
                                   LidarCommandInterface::UNKNOWN));
        // end fpga upgrade, write only
        map.emplace(OB_LIDAR_OPTION_END_FPGA_UPGRADE,
                    std::make_pair(LidarCommandInterface::END_FPGA_UPGRADE,
                                   LidarCommandInterface::UNKNOWN));
        // transfer fpga upgrade package
        map.emplace(
            OB_LIDAR_OPTION_TRANSFER_FPGA_UPGRADE_PACKAGE,
            std::make_pair(LidarCommandInterface::TRANSFER_FPGA_UPGRADE_PACKAGE,
                           LidarCommandInterface::UNKNOWN));
        // start MEMS upgrade, write only
        map.emplace(OB_LIDAR_OPTION_START_MEMS_UPGRADE,
                    std::make_pair(LidarCommandInterface::START_MEMS_UPGRADE,
                                   LidarCommandInterface::UNKNOWN));
        // end MEMS upgrade, write only
        map.emplace(OB_LIDAR_OPTION_END_MEMS_UPGRADE,
                    std::make_pair(LidarCommandInterface::END_MEMS_UPGRADE,
                                   LidarCommandInterface::UNKNOWN));
        // MEMS ID verification, write only
        map.emplace(OB_LIDAR_OPTION_MEMS_ID_VERIFICATION,
                    std::make_pair(LidarCommandInterface::MEMS_ID_VERIFICATION,
                                   LidarCommandInterface::UNKNOWN));
        // send MEMS md5 value, write only
        map.emplace(OB_LIDAR_OPTION_SEND_MEMS_MD5_VALUE,
                    std::make_pair(LidarCommandInterface::SEND_MEMS_MD5_VALUE,
                                   LidarCommandInterface::UNKNOWN));
        // verify MEMS md5 value, write only
        map.emplace(OB_LIDAR_OPTION_VERIFY_MEMS_MD5_VALUE,
                    std::make_pair(LidarCommandInterface::VERIFY_MEMS_MD5_VALUE,
                                   LidarCommandInterface::UNKNOWN));
        // transfer MEMS upgrade package, write only
        map.emplace(
            OB_LIDAR_OPTION_TRANSFER_MEMS_UPGRADE_PACKAGE,
            std::make_pair(LidarCommandInterface::TRANSFER_MEMS_UPGRADE_PACKAGE,
                           LidarCommandInterface::UNKNOWN));
        // product model, read only
        map.emplace(OB_LIDAR_OPTION_PRODUCT_MODEL,
                    std::make_pair(LidarCommandInterface::UNKNOWN,
                                   LidarCommandInterface::GET_PRODUCT_MODEL));
        // firmware version, read only
        map.emplace(
            OB_LIDAR_OPTION_FIRMWARE_VERSION,
            std::make_pair(LidarCommandInterface::UNKNOWN,
                           LidarCommandInterface::GET_FIRMWARE_VERSION));
        // FPGA version, read only
        map.emplace(OB_LIDAR_OPTION_FPGA_VERSION,
                    std::make_pair(LidarCommandInterface::UNKNOWN,
                                   LidarCommandInterface::GET_FPGA_VERSION));
        // spin speed, FIXME: maybe need set
        map.emplace(OB_LIDAR_OPTION_SPIN_SPEED,
                    std::make_pair(LidarCommandInterface::UNKNOWN,
                                   LidarCommandInterface::GET_SPIN_SPEED));

        // MCU temperature, read only
        map.emplace(OB_LIDAR_OPTION_MCU_TEMPERATURE,
                    std::make_pair(LidarCommandInterface::UNKNOWN,
                                   LidarCommandInterface::GET_MCU_TEMPERATURE));

        // FPGA temperature, read only
        map.emplace(
            OB_LIDAR_OPTION_FPGA_TEMPERATURE,
            std::make_pair(LidarCommandInterface::UNKNOWN,
                           LidarCommandInterface::GET_FPGA_TEMPERATURE));

        // FPGA version date, read only
        map.emplace(
            OB_LIDAR_OPTION_FPGA_VERSION_DATE,
            std::make_pair(LidarCommandInterface::UNKNOWN,
                           LidarCommandInterface::GET_FPGA_VERSION_DATE));

        // high voltage, read only
        map.emplace(OB_LIDAR_OPTION_HIGH_VOLTAGE,
                    std::make_pair(LidarCommandInterface::UNKNOWN,
                                   LidarCommandInterface::GET_HIGH_VOLTAGE));

        // TX voltage, read only
        map.emplace(OB_LIDAR_OPTION_TX_VOLTAGE,
                    std::make_pair(LidarCommandInterface::UNKNOWN,
                                   LidarCommandInterface::GET_TX_VOLTAGE));

        return map;
    }
};

}  // namespace ob_lidar
