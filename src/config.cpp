#include <spdlog/spdlog.h>

#include <iostream>
#include <magic_enum/magic_enum.hpp>
#include <nlohmann/json.hpp>
#include <orbbec_lidar/config.hpp>
#include <sstream>
#include <toml++/toml.hpp>

namespace ob_lidar {
// TL2401 default config
static const std::string TL2401_DEVICE_IP = "192.168.1.100";
static constexpr uint16_t TL2401_DEVICE_COMMAND_PORT = 2401;
static constexpr uint16_t TL2401_DEVICE_POINT_CLOUD_PORT = 2401;
static constexpr uint16_t TL2401_DEVICE_IMU_PORT = 2401;
static constexpr uint16_t TL2401_DEVICE_LOG_PORT = 2401;
// other default configs put here

StreamConfig::StreamConfig() = default;

StreamConfig::~StreamConfig() = default;

void StreamConfig::enableStream(const LidarStreamType& stream_type,
                                double frequency_hz) {
    stream_set_.insert(std::make_pair(stream_type, frequency_hz));
}

void StreamConfig::enableIMUStream(double frequency_hz) {
    enableStream(LidarStreamType::IMU, frequency_hz);
}

void StreamConfig::enablePointCloudStream(double frequency_hz) {
    enableStream(LidarStreamType::POINT_CLOUD, frequency_hz);
}

double StreamConfig::getStreamFrequency(
    const LidarStreamType& stream_type) const {
    auto it = stream_set_.find(stream_type);
    if (it != stream_set_.end()) {
        return it->second;
    }
    return 0.0;
}

bool StreamConfig::isStreamEnabled(const LidarStreamType& stream_type) const {
    return stream_set_.find(stream_type) != stream_set_.end();
}

// helper functions
LidarModel parseLidarModel(const std::string& model) {
    if (model == "TL2401") {
        return LidarModel::TL2401;
    } else if (model == "MS600") {
        return LidarModel::MS600;
    }
    // Add other models as needed
    return LidarModel::UNKNOWN;
}

LidarProtocolType parseProtocolType(const std::string& protocol) {
    if (protocol == "udp" || protocol == "UDP") return LidarProtocolType::UDP;
    if (protocol == "tcp" || protocol == "TCP") return LidarProtocolType::TCP;
    return LidarProtocolType::UNKNOWN;
}

LogLevel parseLogLevel(const std::string& level) {
    if (level == "info" || level == "INFO") return LogLevel::INFO;
    if (level == "warning" || level == "WARNING") return LogLevel::WARNING;
    if (level == "error" || level == "ERROR") return LogLevel::ERROR;
    if (level == "trace" || level == "TRACE") return LogLevel::TRACE;
    if (level == "debug" || level == "DEBUG") return LogLevel::DEBUG;
    if (level == "critical" || level == "CRITICAL") return LogLevel::CRITICAL;
    return LogLevel::INFO;
}

NetworkConfig::NetworkConfig() = default;

NetworkConfig::~NetworkConfig() = default;

void NetworkConfig::loadFromJsonString(const std::string& json_config_str) {
    try {
        auto config = nlohmann::json::parse(json_config_str);

        if (config.contains("single_port_mode")) {
            is_single_port_mode_ = config["single_port_mode"].get<bool>();
        } else {
            is_single_port_mode_ = true;  // default value
        }

        if (config.contains("ip")) {
            ip_ = config["ip"].get<std::string>();
        } else {
            ip_ = TL2401_DEVICE_IP;  // default value
        }

        if (config.contains("protocol")) {
            protocol_type_ =
                parseProtocolType(config["protocol"].get<std::string>());
        } else {
            protocol_type_ = parseProtocolType("udp");  // default value
        }

        if (!is_single_port_mode_) {
            if (config.contains("command_port")) {
                port_map_[LidarChannelType::COMMAND] =
                    config["command_port"].get<int>();
            } else {
                port_map_[LidarChannelType::COMMAND] =
                    TL2401_DEVICE_COMMAND_PORT;  // default value
            }

            if (config.contains("point_cloud_port")) {
                port_map_[LidarChannelType::POINT_CLOUD] =
                    config["point_cloud_port"].get<int>();
            } else {
                port_map_[LidarChannelType::POINT_CLOUD] =
                    TL2401_DEVICE_POINT_CLOUD_PORT;  // default value
            }

            if (config.contains("imu_port")) {
                port_map_[LidarChannelType::IMU] =
                    config["imu_port"].get<int>();
            } else {
                port_map_[LidarChannelType::IMU] =
                    TL2401_DEVICE_IMU_PORT;  // default value
            }

            if (config.contains("log_port")) {
                port_map_[LidarChannelType::LOG] =
                    config["log_port"].get<int>();
            } else {
                port_map_[LidarChannelType::LOG] =
                    TL2401_DEVICE_LOG_PORT;  // default value
            }
        } else {
            if (config.contains("port")) {
                port_ = config["port"].get<int>();
            } else {
                port_ = TL2401_DEVICE_COMMAND_PORT;  // default value
            }
        }

    } catch (const nlohmann::json::parse_error& err) {
        std::cerr << "Error parsing JSON config string: " << err.what()
                  << std::endl;
        throw std::runtime_error("Error parsing JSON config string: " +
                                 std::string(err.what()));
    } catch (const nlohmann::json::type_error& err) {
        std::cerr << "Type error in JSON config: " << err.what() << std::endl;
        throw std::runtime_error("Type error in JSON config: " +
                                 std::string(err.what()));
    }
}

NetworkConfig::NetworkConfig(LidarModel model_type,
                             LidarProtocolType network_type) {
    model_ = model_type;
    protocol_type_ = network_type;
    if (model_ == LidarModel::TL2401) {
        ip_ = TL2401_DEVICE_IP;
        port_map_[LidarChannelType::COMMAND] = TL2401_DEVICE_COMMAND_PORT;
        port_map_[LidarChannelType::POINT_CLOUD] =
            TL2401_DEVICE_POINT_CLOUD_PORT;
        port_map_[LidarChannelType::IMU] = TL2401_DEVICE_IMU_PORT;
        port_map_[LidarChannelType::LOG] = TL2401_DEVICE_LOG_PORT;
    } else {
        // throw error
        const auto err_msg = spdlog::fmt_lib::format(
            "Unsupported model: {}", magic_enum::enum_name(model_));
        throw std::invalid_argument(err_msg);
    }
}

LidarProtocolType NetworkConfig::getProtocolType() const {
    return protocol_type_;
}

std::string NetworkConfig::getIp() const { return ip_; }

uint16_t NetworkConfig::getPort(LidarChannelType channel_type) const {
    return port_map_.at(channel_type);
}

uint16_t NetworkConfig::getPort() const { return port_; }

bool NetworkConfig::hasChannel(LidarChannelType channel_type) const {
    return port_map_.find(channel_type) != port_map_.end();
}

bool NetworkConfig::isSinglePort() const {
    return port_map_.size() == 1 || is_single_port_mode_;
}

#ifdef _WIN32
#include <direct.h>
#define GetCurrentDir _getcwd
#else
#include <unistd.h>
#define GetCurrentDir getcwd
#endif

LoggerConfig::LoggerConfig() {
    char buffer[1024];
    if (GetCurrentDir(buffer, sizeof(buffer)) == nullptr) {
        std::cerr << "LoggerConfig: failed to get current directory"
                  << std::endl;
        throw std::runtime_error(
            "LoggerConfig: failed to get current "
            "directory");
    }
    log_file_dir_ = std::string(buffer) + "/logs";
}

LoggerConfig::LoggerConfig(const std::string& config_file_path) {
    try {
        auto config = toml::parse_file(config_file_path);
        if (auto logging = config["logging"]) {
            max_log_file_size_ =
                logging["max_file_size"].value_or(10 * 1024 * 1024);
            max_log_file_num_ = logging["max_file_num"].value_or(10);
            log_file_dir_ = logging["log_file_dir"].value_or("/tmp");
            console_log_level_ =
                parseLogLevel(logging["console_log_level"].value_or("info"));
            file_log_level_ =
                parseLogLevel(logging["file_log_level"].value_or("info"));
            log_to_console_ = logging["log_to_console"].value_or(true);
            log_to_file_ = logging["log_to_file"].value_or(true);
        }
    } catch (const toml::parse_error& err) {
        std::cerr << "parsing config file: " << config_file_path << " failed"
                  << std::endl;
        std::cerr << "Error parsing config file: " << err.description()
                  << std::endl;
        throw std::runtime_error("Error parsing config file: " +
                                 std::string(err.description()));
    }
}

void LoggerConfig::loadFromJsonString(const std::string& json_config_str) {
    try {
        auto config = nlohmann::json::parse(json_config_str);
        if (config.contains("max_file_size")) {
            max_log_file_size_ = config["max_file_size"].get<uint32_t>();
        }
        if (config.contains("max_file_num")) {
            max_log_file_num_ = config["max_file_num"].get<uint32_t>();
        }
        if (config.contains("log_file_dir")) {
            log_file_dir_ = config["log_file_dir"].get<std::string>();
        }
        if (config.contains("console_log_level")) {
            console_log_level_ =
                parseLogLevel(config["console_log_level"].get<std::string>());
        }
        if (config.contains("file_log_level")) {
            file_log_level_ =
                parseLogLevel(config["file_log_level"].get<std::string>());
        }
        if (config.contains("log_to_console")) {
            log_to_console_ = config["log_to_console"].get<bool>();
        }
        if (config.contains("log_to_file")) {
            log_to_file_ = config["log_to_file"].get<bool>();
        }
    } catch (const nlohmann::json::parse_error& err) {
        std::cerr << "Error parsing JSON config string: " << err.what()
                  << std::endl;
        throw std::runtime_error("Error parsing JSON config string: " +
                                 std::string(err.what()));
    } catch (const nlohmann::json::type_error& err) {
        std::cerr << "Type error in JSON config: " << err.what() << std::endl;
        throw std::runtime_error("Type error in JSON config: " +
                                 std::string(err.what()));
    }
}

LoggerConfig::LoggerConfig(uint32_t max_log_file_size,
                           uint32_t max_log_file_num,
                           const std::string& log_file_dir)
    : max_log_file_size_(max_log_file_size),
      max_log_file_num_(max_log_file_num),
      log_file_dir_(log_file_dir) {}

uint32_t LoggerConfig::getMaxLogFileSize() const { return max_log_file_size_; }

uint32_t LoggerConfig::getMaxLogFileNum() const { return max_log_file_num_; }

std::string LoggerConfig::getLogFileDir() const { return log_file_dir_; }

LogLevel LoggerConfig::getConsoleLogLevel() const { return console_log_level_; }

LogLevel LoggerConfig::getFileLogLevel() const { return file_log_level_; }

bool LoggerConfig::shouldLogToConsole() const { return log_to_console_; }

bool LoggerConfig::shouldLogToFile() const { return log_to_file_; }

std::string LoggerConfig::getDeviceName() const { return device_name_; }

void LoggerConfig::setDeviceName(const std::string& device_name) {
    device_name_ = device_name;
}

bool LoggerConfig::enableAsync() const { return enable_async_; }

DeviceConfig::DeviceConfig(const std::string& config_file_path) {
    try {
        auto config = toml::parse_file(config_file_path);

        // Parse device section
        if (const auto device_table = config["device"].as_table()) {
            for (const auto& [_, device_config_node] : *device_table) {
                std::stringstream ss;
                ss << toml::json_formatter(device_config_node);
                const auto json_config_str = ss.str();
                try {
                    auto json = nlohmann::json::parse(json_config_str);
                    device_name_ = json["name"].get<std::string>();
                    model_ = parseLidarModel(json["model"].get<std::string>());
                    auto network_config = json["network"];
                    network_config_ = std::make_shared<NetworkConfig>();
                    network_config_->loadFromJsonString(network_config.dump(2));
                    protocol_type_ = network_config_->getProtocolType();
                } catch (const nlohmann::json::parse_error& err) {
                    std::cerr
                        << "Error parsing JSON config string: " << err.what()
                        << std::endl;
                    throw std::runtime_error(
                        "Error parsing JSON config string: " +
                        std::string(err.what()));
                } catch (const nlohmann::json::type_error& err) {
                    std::cerr << "Type error in JSON config: " << err.what()
                              << std::endl;
                    throw std::runtime_error("Type error in JSON config: " +
                                             std::string(err.what()));
                }
            }
        }

        // Parse logging section
        if (auto logging = config["logging"]) {
            logger_config_ = std::make_shared<LoggerConfig>(config_file_path);
        }
    } catch (const toml::parse_error& err) {
        std::cerr << "Error parsing config file: " << err.description()
                  << std::endl;
        throw std::runtime_error("Error parsing config file: " +
                                 std::string(err.description()));
    }
}

DeviceConfig::~DeviceConfig() = default;

DeviceConfig::DeviceConfig(std::string device_name,
                           std::shared_ptr<NetworkConfig> network_config,
                           std::shared_ptr<LoggerConfig> logger_config)
    : device_name_(std::move(device_name)),
      network_config_(std::move(network_config)),
      logger_config_(std::move(logger_config)) {}

DeviceConfig::DeviceConfig(std::string device_name, LidarModel model,
                           LidarProtocolType protocol_type)
    : device_name_(std::move(device_name)),
      model_(model),
      protocol_type_(protocol_type),
      option_timeout_(std::chrono::milliseconds(100)) {}

void DeviceConfig::setNetworkConfig(
    std::shared_ptr<NetworkConfig> network_config) {
    network_config_ = std::move(network_config);
}

void DeviceConfig::setLoggerConfig(
    std::shared_ptr<LoggerConfig> logger_config) {
    logger_config_ = std::move(logger_config);
}

std::string DeviceConfig::getDeviceName() const { return device_name_; }

std::shared_ptr<NetworkConfig> DeviceConfig::getNetworkConfig() const {
    return network_config_;
}

std::shared_ptr<LoggerConfig> DeviceConfig::getLoggerConfig() const {
    return logger_config_;
}

void DeviceConfig::setOptionTimeout(std::chrono::milliseconds timeout) {
    option_timeout_ = timeout;
}

std::chrono::milliseconds DeviceConfig::getOptionTimeout() const {
    return option_timeout_;
}

LidarModel DeviceConfig::getModel() const { return model_; }

LidarProtocolType DeviceConfig::getProtocolType() const {
    return protocol_type_;
}

DeviceConfigBuilder::DeviceConfigBuilder() = default;

DeviceConfigBuilder::~DeviceConfigBuilder() = default;

DeviceConfigBuilder& DeviceConfigBuilder::setDeviceName(
    const std::string& device_name) {
    device_name_ = device_name;
    return *this;
}

DeviceConfigBuilder& DeviceConfigBuilder::setModel(const std::string& model) {
    model_ = parseLidarModel(model);
    return *this;
}

DeviceConfigBuilder& DeviceConfigBuilder::setProtocolType(
    const std::string& protocol_type) {
    protocol_type_ = parseProtocolType(protocol_type);

    return *this;
}

DeviceConfigBuilder& DeviceConfigBuilder::setNetworkConfigJsonString(
    const std::string& json_config_str) {
    net_work_config_json_str_ = json_config_str;
    network_config_ = std::make_shared<NetworkConfig>();
    network_config_->loadFromJsonString(json_config_str);
    protocol_type_ = network_config_->getProtocolType();
    return *this;
}

DeviceConfigBuilder& DeviceConfigBuilder::setLoggerConfigJsonString(
    const std::string& json_config_str) {
    logger_config_json_str_ = json_config_str;
    logger_config_ = std::make_shared<LoggerConfig>();
    logger_config_->loadFromJsonString(json_config_str);
    return *this;
}

std::shared_ptr<DeviceConfig> DeviceConfigBuilder::build() {
    // check if all required fields are set
    if (device_name_.empty() || model_ == LidarModel::UNKNOWN) {
        std::string error_msg =
            "You must set device name, model  before "
            "building device config";
        std::cerr << error_msg << std::endl;
        throw std::invalid_argument(error_msg);
    }

    if (network_config_ == nullptr &&
        protocol_type_ != LidarProtocolType::UNKNOWN) {
        network_config_ = getDefaultNetworkConfig(model_, protocol_type_);
    } else if (network_config_ == nullptr) {
        std::string error_msg =
            "DeviceConfigBuilder: you must set network config json string or "
            "protocol type";
        std::cerr << error_msg << std::endl;
        throw std::invalid_argument(error_msg);
    }

    // create device config
    auto device_config =
        std::make_shared<DeviceConfig>(device_name_, model_, protocol_type_);

    if (network_config_ == nullptr) {
        std::cerr << "Network config is not set, using default network config "
                     "for model: "
                  << magic_enum::enum_name(model_) << ", protocol type: "
                  << magic_enum::enum_name(protocol_type_) << std::endl;
        network_config_ = getDefaultNetworkConfig(model_, protocol_type_);
    }
    if (logger_config_ == nullptr) {
        std::cerr << "Logger config is not set, using default logger config"
                  << std::endl;
        logger_config_ = getDefaultLoggerConfig();
    }
    device_config->setNetworkConfig(network_config_);
    device_config->setLoggerConfig(logger_config_);
    if (!checkDeviceConfig()) {
        std::string error_msg = "DeviceConfigBuilder: device config is invalid";
        std::cerr << error_msg << std::endl;
        throw std::invalid_argument(error_msg);
    }
    return device_config;
}

bool DeviceConfigBuilder::checkDeviceConfig() const {
    if (network_config_ == nullptr) {
        std::cerr << "DeviceConfigBuilder: network config is not set"
                  << std::endl;
        return false;
    }
    if (logger_config_ == nullptr) {
        std::cerr << "DeviceConfigBuilder: logger config is not set"
                  << std::endl;
        return false;
    }
    if (network_config_->getProtocolType() != protocol_type_) {
        std::cerr << "DeviceConfigBuilder: protocol type mismatch" << std::endl;
        return false;
    }
    return true;
}

std::shared_ptr<NetworkConfig> DeviceConfigBuilder::getDefaultNetworkConfig(
    LidarModel model, LidarProtocolType protocol_type) {
    auto network_config = std::make_shared<NetworkConfig>(model, protocol_type);
    return network_config;
}

std::shared_ptr<LoggerConfig> DeviceConfigBuilder::getDefaultLoggerConfig() {
    return std::make_shared<LoggerConfig>();
}
}  // namespace ob_lidar
