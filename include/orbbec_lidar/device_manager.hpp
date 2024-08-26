#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "config.hpp"
#include "device.hpp"
#include "filter.hpp"
#include "frame.hpp"
#include "option.hpp"
#include "recorder.hpp"
#include "types.hpp"

namespace ob_lidar {

using onDeviceConnectedCallback = std::function<void(std::shared_ptr<Device>)>;
using onDeviceDisconnectedCallback =
    std::function<void(std::shared_ptr<Device>)>;
namespace detail {
class DeviceManagerImpl;
}  // namespace detail

class DeviceManager {
   public:
    /**
     * \brief Default constructor for the Driver class.
     */
    DeviceManager();

    /**
     * \brief Constructs a Driver object with the specified configuration file
     * path.
     *
     * \param config_file_path The path to the configuration file.
     */
    explicit DeviceManager(const std::string &config_file_path);

    /**
     * \brief Enables or disables device discovery.
     *
     * \param enable A boolean flag to enable (true) or disable (false)
     * discovery.
     */
    void enableDiscovery(bool enable);

    /**
     * \brief Sets the callback functions for device connection and
     * disconnection events.
     *
     * \param on_device_connected Callback function to be called when a device
     * is connected.
     * \param on_device_disconnected Callback function to be
     * called when a device is disconnected.
     */
    void setOnDeviceChangedCallback(
        const onDeviceConnectedCallback &on_device_connected,
        const onDeviceDisconnectedCallback &on_device_disconnected) const;

    ~DeviceManager();

    /**
     * \brief Retrieves a list of devices.
     *
     * \return A vector of shared pointers to Device objects. return empty
     * vector if no devices found
     */
    std::vector<std::shared_ptr<Device>> getDevices();
    /**
     * \brief Retrieves a device by its name.
     *
     * \param device_name The unique name of the device to retrieve.
     * \return A shared pointer to the Device object. return nullptr if device
     * not found
     */
    std::shared_ptr<Device> getDevice(const std::string &device_name);

    /**
     * \brief Adds a device to the driver using the specified configuration.
     *
     * \param config The configuration of the device to add.
     */
    void addDevice(std::shared_ptr<DeviceConfig> config);

    /**
     * \brief Removes a device from the driver by its name.
     *
     * \param device_name The name of the device to remove.
     */
    void removeDevice(const std::string &device_name);
    /**
     * \brief Starts all devices managed by the driver.
     */
    void start();
    /**
     * \brief Stops all devices managed by the driver.
     */
    void stop();

   private:
    std::unique_ptr<detail::DeviceManagerImpl> impl_;
};
}  // namespace ob_lidar
