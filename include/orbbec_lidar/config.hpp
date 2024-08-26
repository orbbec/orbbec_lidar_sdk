#pragma once
#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <string>

#include "types.hpp"

namespace ob_lidar {
class StreamConfig {
   public:
    /**
     * @brief Constructs a StreamConfig object.
     *
     * This constructor initializes a StreamConfig object with default settings.
     */
    StreamConfig();

    ~StreamConfig();

    /**
     * @brief Enables a specific stream with the given frequency.
     *
     * This function enables a specific Lidar stream type with the specified
     * frequency in Hz.
     *
     * @param stream_type The type of the Lidar stream to enable.
     * @param frequency_hz The frequency at which the stream should operate, in
     * Hertz.
     */
    void enableStream(const LidarStreamType& stream_type, double frequency_hz);
    /**
     * @brief Enables the IMU stream with the given frequency.
     *
     * This function enables the IMU stream with the specified frequency in Hz.
     *
     * @param frequency_hz The frequency at which the IMU stream should operate,
     * in Hertz. Hertz will be covert to motor speed
     */
    void enableIMUStream(double frequency_hz);
    /**
     * @brief Enables the point cloud stream with the given frequency.
     *
     * This function enables the point cloud stream with the specified frequency
     * in Hz.
     *
     * @param frequency_hz The frequency at which the point cloud stream should
     * operate, in Hertz.
     */
    void enablePointCloudStream(double frequency_hz);

    /**
     * @brief Gets the frequency of an enabled stream.
     *
     * This function returns the frequency of a specific enabled Lidar stream
     * type.
     *
     * @param stream_type The type of the Lidar stream to query.
     * @return The frequency of the stream in Hz, or 0 if the stream is not
     * enabled.
     */
    double getStreamFrequency(const LidarStreamType& stream_type) const;

    /**
     * @brief Checks if a specific lidar stream type is enabled.
     *
     * This function checks whether a specific lidar stream type is enabled or
     * not.
     *
     * @param stream_type The lidar stream type to check.
     * @return `true` if the stream type is enabled, `false` otherwise.
     */
    bool isStreamEnabled(const LidarStreamType& stream_type) const;

   private:
    std::map<LidarStreamType, double> stream_set_;
};

class NetworkConfig {
   public:
    NetworkConfig();

    ~NetworkConfig();

    /**
     * @brief Loads the configuration from a JSON string.
     *
     * This function parses a JSON string and loads the configuration settings
     * from it. The JSON string should contain valid configuration parameters
     * in the expected format.
     *
     * @param json_config_str The JSON string containing the configuration.
     */
    void loadFromJsonString(const std::string& json_config_str);
    /**
     * @brief Constructs a NetworkConfig object with the specified model type
     * and network type.
     *
     * This constructor initializes a NetworkConfig object by setting the lidar
     * model type and the network protocol type.
     *
     * @param model_type The type of the lidar model.
     * @param protocol_type The type of the network protocol to be used.
     */
    NetworkConfig(LidarModel model_type, LidarProtocolType protocol_type);

    /**
     * @brief Gets the network type.
     *
     * This function returns the type of the network protocol used by the
     * device.
     *
     * @return The network protocol type.
     */
    LidarProtocolType getProtocolType() const;

    /**
     * @brief Get the IP address.
     *
     * This function returns the IP address associated with the configuration.
     *
     * @return The IP address as a string.
     */
    std::string getIp() const;
    /**
     * @brief Gets the port number for the specified channel type.
     *
     * This function returns the port number associated with the given Lidar
     * channel type.
     *
     * @param channel_type The type of the Lidar channel.
     * @return The port number as a 16-bit unsigned integer.
     */

    uint16_t getPort(LidarChannelType channel_type) const;

    uint16_t getPort() const;
    /**
     * @brief Checks if the specified channel type exists.
     *
     * This function checks whether the given Lidar channel type is present
     * in the configuration.
     *
     * @param channel_type The type of the Lidar channel to check.
     * @return True if the channel type exists, false otherwise.
     */
    bool hasChannel(LidarChannelType channel_type) const;

    /**
     * @brief Checks if the driver is configured to use a single port.
     *
     * @return true if the driver is configured to use a single port, false
     * otherwise.
     */
    bool isSinglePort() const;

   private:
    LidarModel model_ = LidarModel::UNKNOWN;
    LidarProtocolType protocol_type_ = LidarProtocolType::UNKNOWN;
    std::string ip_ = "192.168.1.100";
    std::map<LidarChannelType, uint16_t> port_map_;
    bool is_single_port_mode_ = true;
    uint16_t port_ = 2401;
};

class LoggerConfig {
   public:
    LoggerConfig();

    LoggerConfig(const std::string& config_file_path);

    /**
     * @brief Constructs a LoggerConfig object with the specified parameters.
     *
     * This constructor initializes a LoggerConfig object with the specified
     * maximum log file size, maximum number of log files, and log file
     * directory.
     *
     * @param max_log_file_size The maximum size of a log file in bytes.
     * @param max_log_file_num The maximum number of log files to retain.
     * @param log_file_dir The directory where log files will be stored.
     */
    LoggerConfig(uint32_t max_log_file_size, uint32_t max_log_file_num,
                 const std::string& log_file_dir);

    ~LoggerConfig() = default;

    /**
     * @brief Loads the configuration from a JSON string.
     *
     * This function parses a JSON string and loads the configuration settings
     * into the object. The JSON string should follow a specific format that
     * matches the expected configuration structure.
     *
     * @param json_config_str The JSON string containing the configuration.
     */
    void loadFromJsonString(const std::string& json_config_str);

    /**
     * @brief Gets the maximum log file size.
     *
     * This function returns the maximum size of a log file in bytes.
     *
     * @return The maximum log file size in bytes.
     */
    uint32_t getMaxLogFileSize() const;

    /**
     * @brief Gets the maximum number of log files.
     *
     * This function returns the maximum number of log files to retain.
     *
     * @return The maximum number of log files.
     */
    uint32_t getMaxLogFileNum() const;

    /**
     * @brief Gets the log file directory.
     *
     * This function returns the directory where log files are stored.
     *
     * @return The log file directory as a string.
     */
    std::string getLogFileDir() const;

    /**
     * @brief Gets the log level.
     *
     * This function returns the log level for the console.
     *
     * @return The log level.
     */
    LogLevel getConsoleLogLevel() const;

    /**
     * Retrieves the log level for the file.
     *
     * @return The log level for the file.
     */
    LogLevel getFileLogLevel() const;

    /**
     * @brief Checks whether the log messages should be printed to the console.
     *
     * @return true if the log messages should be printed to the console, false
     * otherwise.
     */
    bool shouldLogToConsole() const;

    /**
     * \brief Checks whether logging to a file is enabled.
     *
     * \return true if logging to a file is enabled, false otherwise.
     */
    bool shouldLogToFile() const;

    /**
     * @brief Gets the device name.
     *
     * This function returns the name of the device.
     *
     * @return The device name as a string.
     */
    std::string getDeviceName() const;

    /**
     * @brief Sets the device name for the Orb Lidar driver.
     *
     * This function sets the device name to be used by the Orb Lidar driver.
     * The device name should be a string representing the name or identifier of
     * the specific device to be used.
     *
     * @param device_name The device name to be set.
     */
    void setDeviceName(const std::string& device_name);

    /**
     * @brief Returns whether asynchronous mode is enabled.
     *
     * This function returns a boolean value indicating whether asynchronous
     * mode is enabled or not.
     *
     * @return True if asynchronous mode is enabled, false otherwise.
     */
    bool enableAsync() const;

   private:
    uint32_t max_log_file_size_ = 10 * 1024 * 1024;  // default is 10M
    uint32_t max_log_file_num_ = 10;                 // default is 10
    std::string log_file_dir_ = "./";                // default is current dir
    LogLevel console_log_level_ = LogLevel::INFO;
    LogLevel file_log_level_ = LogLevel::INFO;
    bool log_to_console_ = true;
    bool log_to_file_ = true;
    std::string device_name_ = "ob_lidar";
    bool enable_async_ = false;
};

class DeviceConfig {
   public:
    explicit DeviceConfig(const std::string& config_file_path);

    ~DeviceConfig();

    /**
     * @brief Constructs a DeviceConfig object.
     *
     * @param device_name The name of the device.
     * @param network_config A shared pointer to the NetworkConfig object.
     * @param logger_config A shared pointer to the LoggerConfig object.
     */
    DeviceConfig(std::string device_name,
                 std::shared_ptr<NetworkConfig> network_config,
                 std::shared_ptr<LoggerConfig> logger_config);

    /**
     * @brief Constructs a DeviceConfig object.
     *
     * This constructor initializes a DeviceConfig object with the specified
     * device name, lidar model, and lidar network type.
     *
     * @param device_name The name of the device.
     * @param model The lidar model.
     * @param protocol_type The lidar network type
     */
    DeviceConfig(std::string device_name, LidarModel model,
                 LidarProtocolType protocol_type);

    /**
     * @brief Get the device name.
     *
     * This function returns the name of the device.
     *
     * @return The device name as a string.
     */
    std::string getDeviceName() const;

    /**
     * @brief Gets the network configuration.
     *
     * This function returns a reference to the network configuration object.
     *
     * @return A shared pointer to the NetworkConfig object.
     */
    std::shared_ptr<NetworkConfig> getNetworkConfig() const;

    /**
     * @brief Retrieves the logger configuration.
     *
     * This function returns a reference to the logger configuration object.
     * The logger configuration contains settings related to logging behavior,
     * such as log levels, log file paths, etc.
     *
     * @return A shared pointer to the logger configuration object.
     */
    std::shared_ptr<LoggerConfig> getLoggerConfig() const;

    /**
     * @brief Sets the network configuration for the Orb Lidar driver.
     *
     * This function sets the network configuration for the Orb Lidar driver.
     * The network configuration includes parameters such as IP address, port
     * number, and other network-related settings.
     *
     * @param network_config A shared pointer to a NetworkConfig object that
     * contains the network configuration.
     */
    void setNetworkConfig(std::shared_ptr<NetworkConfig> network_config);

    /**
     * @brief Sets the logger configuration for the Orb Lidar driver.
     *
     * This function sets the logger configuration for the Orb Lidar driver. The
     * logger configuration includes parameters such as log file path, log file
     * size, and other logging-related settings.
     *
     * @param logger_config A shared pointer to a LoggerConfig object that
     * contains the logger configuration.
     */
    void setLoggerConfig(std::shared_ptr<LoggerConfig> logger_config);

    /**
     * Sets the timeout for an option.
     *
     * This function sets the timeout for an option. The timeout specifies the
     * maximum time to wait for a response when setting an option. If the
     * timeout is exceeded, an error will be returned.
     *
     * @param timeout The timeout duration in milliseconds.
     */
    void setOptionTimeout(std::chrono::milliseconds timeout);

    /**
     * Retrieves the timeout duration for retrieving options.
     *
     * @return The timeout duration in milliseconds.
     */
    [[nodiscard]] std::chrono::milliseconds getOptionTimeout() const;

    /**
     * @brief Get the model of the lidar.
     *
     * This function returns the model of the lidar.
     *
     * @return The model of the lidar.
     */
    LidarModel getModel() const;

    /**
     * @brief Gets the protocol type of the lidar.
     *
     * @return The protocol type of the lidar.
     */
    LidarProtocolType getProtocolType() const;

   private:
    // a unique name for the device
    std::string device_name_;
    // product model MS600 or TL2401
    LidarModel model_ = LidarModel::UNKNOWN;
    // network protocol type
    LidarProtocolType protocol_type_ = LidarProtocolType::UNKNOWN;
    // set/get option timeout
    std::chrono::milliseconds option_timeout_{1000};
    // network config
    std::shared_ptr<NetworkConfig> network_config_ = nullptr;
    // logger config
    std::shared_ptr<LoggerConfig> logger_config_ = nullptr;
};

/**
 * @brief The DeviceConfigBuilder class is used to build a DeviceConfig object.
 *
 * The DeviceConfigBuilder class provides a convenient way to construct a
 * DeviceConfig object with various configurations. It follows the Builder
 * design pattern, allowing the user to set different properties of the
 * DeviceConfig object before building it.
 *
 * To use the DeviceConfigBuilder, create an instance of the class and call the
 * appropriate setter methods to set the desired configurations. Finally, call
 * the `build()` method to obtain a shared pointer to the constructed
 * DeviceConfig object.
 *
 * If only the device name, model, and protocol type are set, default values
 * will be used for the network and logger configurations. To specify custom
 * network configurations, use the setNetworkConfigJsonString method with a JSON
 * string containing the desired settings. In this case, the protocol type set
 * separately will be ignored, as it should be included in the JSON
 * configuration.
 *
 * Example usage with default network and logger configurations:
 *
 * @code
 * DeviceConfigBuilder builder;
 * builder.setDeviceName("MyLidar")
 *        .setModel("MS600")
 *        .setProtocolType("TCP");
 * std::shared_ptr<DeviceConfig> config = builder.build();
 * @endcode
 *
 * Example usage with custom network configuration:
 *
 * @code
 * DeviceConfigBuilder builder;
 * builder.setDeviceName("MyLidar")
 *        .setModel("MS600")
 *        .setNetworkConfigJsonString(network_json_config_str)
 *        .setLoggerConfigJsonString(logger_json_config_str);
 * std::shared_ptr<DeviceConfig> config = builder.build();
 * @endcode
 *
 * Note: When using setNetworkConfigJsonString, the protocol type should be
 * specified within the JSON configuration, and setProtocolType should not be
 * called separately.
 */

class DeviceConfigBuilder {
   public:
    /**
     * @brief Constructs a DeviceConfigBuilder object.
     *
     * This constructor initializes a DeviceConfigBuilder object with default
     * settings.
     */
    DeviceConfigBuilder();

    ~DeviceConfigBuilder();
    /**
     * @brief Sets the device name for the DeviceConfigBuilder.
     *
     * This function sets the device name for the DeviceConfigBuilder object.
     *
     * @param device_name The device name to be set.
     * @return A reference to the DeviceConfigBuilder object.
     */
    DeviceConfigBuilder& setDeviceName(const std::string& device_name);

    /**
     * @brief Sets the model of the lidar device.
     *
     * This function is used to set the model of the lidar device in the
     * configuration.
     *
     * @param model The model of the lidar device.
     * @return A reference to the DeviceConfigBuilder object.
     */
    DeviceConfigBuilder& setModel(const std::string& model);

    /**
     * @brief Sets the network type for the device configuration.
     *
     * This function allows you to specify the network type for the device
     * configuration. The network type determines the type of network connection
     * used by the lidar device.
     *
     * @param protocol_type The network protocol type to set.
     * @return A reference to the DeviceConfigBuilder object.
     */
    DeviceConfigBuilder& setProtocolType(const std::string& protocol_type);

    /**
     * @brief Sets the network configuration JSON string.
     *
     * This function sets the network configuration JSON string for the device.
     *
     * @param json_config_str The JSON string containing the network
     * configuration.
     * @return A reference to the DeviceConfigBuilder object.
     */
    DeviceConfigBuilder& setNetworkConfigJsonString(
        const std::string& json_config_str);

    /**
     * @brief Sets the logger configuration JSON string.
     *
     * This function sets the logger configuration JSON string for the device.
     *
     * @param json_config_str The JSON string containing the logger
     * configuration.
     * @return A reference to the DeviceConfigBuilder object.
     */
    DeviceConfigBuilder& setLoggerConfigJsonString(
        const std::string& json_config_str);

    /**
     * @brief Builds the DeviceConfig object.
     *
     * This function builds the DeviceConfig object with the specified
     * configurations.
     *
     * @return A shared pointer to the DeviceConfig object.
     */
    std::shared_ptr<DeviceConfig> build();

   private:
    bool checkDeviceConfig() const;

    static std::shared_ptr<NetworkConfig> getDefaultNetworkConfig(
        LidarModel model, LidarProtocolType protocol_type);

    std::shared_ptr<LoggerConfig> getDefaultLoggerConfig();

   private:
    std::string device_name_;
    LidarModel model_ = LidarModel::UNKNOWN;
    LidarProtocolType protocol_type_ = LidarProtocolType::UNKNOWN;
    std::shared_ptr<NetworkConfig> network_config_ = nullptr;
    std::shared_ptr<LoggerConfig> logger_config_ = nullptr;
    std::string net_work_config_json_str_;
    std::string logger_config_json_str_;
};
}  // namespace ob_lidar
