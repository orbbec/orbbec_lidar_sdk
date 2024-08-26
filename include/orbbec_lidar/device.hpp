#pragma once
#include <chrono>
#include <memory>
#include <string>

#include "config.hpp"
#include "frame.hpp"
#include "option.hpp"
#include "types.hpp"

namespace ob_lidar {

class Device;

using onDeviceConnectedCallback = std::function<void(std::shared_ptr<Device>)>;
using onDeviceDisconnectedCallback =
    std::function<void(std::shared_ptr<Device>)>;
namespace detail {
class DeviceImpl;
}  // namespace detail

template <typename T>
struct always_false : std::false_type {};

template <typename T>
constexpr bool always_false_v = always_false<T>::value;
class Device {
   public:
    /**
     * \brief Constructs a Device object with the given implementation. do not
     * give use explicitly call this constructor.
     *
     * \param impl A unique pointer to the implementation of the device.
     */
    explicit Device(std::unique_ptr<detail::DeviceImpl> impl);

    ~Device() noexcept;

    /**
     * \brief Retrieves information about the device.
     *
     * \return A DeviceInfo object containing information about the device.
     */

    DeviceInfo getInfo();

    /**
     * \brief Starts the device.
     *
     * \return Status indicating the success or failure of the operation.
     */
    Status start();
    /**
     * \brief Stops the device.
     *
     * \return Status indicating the success or failure of the operation.
     */
    Status stop();
    /**
     * \brief Starts streaming data with the given configuration and callback.
     *
     * \param config The configuration for the stream.
     * \param callback The callback function to be called with each frame.
     * \return Status indicating the success or failure of the operation.
     */
    Status start(const std::shared_ptr<StreamConfig> &config,
                 const FrameCallback &callback);

    /**
     * \brief Registers a frame observer with the given ID and callback
     * function.
     *
     * \param observer_id The ID of the observer to register.
     * \param callback The callback function to be called with each frame.
     */
    void registerFrameObserver(int observer_id, const FrameCallback &callback);
    /**
     * \brief Unregisters a frame observer with the given ID.
     *
     * \param observer_id The ID of the observer to unregister.
     */
    void unregisterFrameObserver(int observer_id);
    /**
     * Sets the integer value of a specified option.
     *
     * @param option The LidarOption enum value representing the option to set.
     * @param value The integer value to set for the specified option.
     * @return The status of the operation.
     */
    Status setIntOption(const LidarOption &option, int value);

    /**
     * Sets the specified option to the given uint16_t value.
     *
     * @param option The LidarOption to set.
     * @param value The uint16_t value to set the option to.
     * @return The status of the operation.
     */
    Status setUint16Option(const LidarOption &option, uint16_t value);

    /**
     * Sets the boolean value of a specified option.
     *
     * @param option The LidarOption enum value representing the option to set.
     * @param value The boolean value to set for the specified option.
     * @return The status of the operation.
     */
    Status setBoolOption(const LidarOption &option, bool value);

    /**
     * Sets the float value of a specified option.
     *
     * @param option The LidarOption enum value representing the option to set.
     * @param value The float value to set for the specified option.
     * @return The status of the operation.
     */
    Status setFloatOption(const LidarOption &option, float value);

    /**
     * Sets the struct value of a specified option.
     *
     * @param option The LidarOption enum value representing the option to set.
     * @param value The struct value to set for the specified option.
     * @return The status of the operation.
     */
    Status setOption(const LidarOption &option, const void *value, size_t size);

    /**
     * Sets an option for the device at the specified address.
     *
     * @param address The address of the option to set.
     * @param value A pointer to the value to set for the option.
     * @param value_size The size of the value in bytes.
     * @return The status of the operation.
     */
    Status setOption(const uint16_t &address, const void *value,
                     size_t value_size);

    /**
     * Sets the value of a specific option for the lidar device.
     *
     * @tparam option The option to set.
     * @param value The value to set for the option.
     * @return The status of the operation.
     */
    template <LidarOption option>
    Status setOption(typename LidarOptionTrait<option>::type value) {
        using ExpectedType = typename LidarOptionTrait<option>::type;
        if constexpr (std::is_same_v<ExpectedType, int32_t> ||
                      std::is_same_v<ExpectedType, uint32_t>) {
            return setIntOption(option, value);
        } else if constexpr (std::is_same_v<ExpectedType, int16_t> ||
                             std::is_same_v<ExpectedType, uint16_t>) {
            return setUint16Option(option, value);
        } else if constexpr (std::is_same_v<ExpectedType, std::string>) {
            return setOption(option, value.c_str(), value.size());
        } else if constexpr (std::is_same_v<ExpectedType, bool>) {
            return setBoolOption(option, value);
        } else if constexpr (std::is_same_v<ExpectedType, float>) {
            return setFloatOption(option, value);
        } else {
            static_assert(sizeof(ExpectedType) == sizeof(value),
                          "Invalid struct size");
            // check if the struct is pod
            if constexpr (std::is_pod_v<ExpectedType>) {
                return setStructOption(option, &value, sizeof(ExpectedType));
            } else {
                static_assert(always_false_v<ExpectedType>,
                              "Struct is not POD");
            }
        }
    }

    /**
     * Retrieves the integer value of a specified option.
     *
     * @param option The LidarOption enum value representing the option to get.
     * @return The integer value of the specified option.You don't need to
     * convert to host order, the value is already in host order.
     */
    int getIntOption(const LidarOption &option);

    /**
     * Retrieves the uint16_t value of a specified option.
     *
     * @param option The LidarOption enum value representing the option to get.
     * @return The uint16_t value of the specified option.You don't need to
     * convert to host order, the value is already in host order.
     */
    uint16_t getUint16Option(const LidarOption &option);

    /**
     * Retrieves the boolean value of a specified option.
     *
     * @param option The LidarOption enum value representing the option to get.
     * @return The boolean value of the specified option.
     */
    bool getBoolOption(const LidarOption &option);

    /**
     * Retrieves the float value of a specified option.
     *
     * @param option The LidarOption enum value representing the option to get.
     * @return The float value of the specified option.
     */
    float getFloatOption(const LidarOption &option);

    /**
     * Retrieves the value of a specific option from the Lidar device.
     *
     * @param option The Lidar option to retrieve.
     * @param value Pointer to a memory location where the retrieved value will
     * be stored.
     * @param size The size of the memory location pointed to by `value`.
     * @param size_read Pointer to a variable that will store the actual size of
     * the retrieved value. You need to convert the value to host order.
     * @return The status of the operation.
     */
    Status getOption(const LidarOption &option, void *value, size_t size,
                     size_t *size_read);

    /**
     * Retrieves the value of an option from the device.
     *
     * @param address The address of the option to retrieve.
     * @param value A pointer to the memory location where the retrieved value
     * will be stored.
     * @param size The size of the memory location pointed to by `value`.
     * @param size_read A pointer to a variable that will store the actual size
     * of the retrieved value. This parameter can be set to `nullptr` if the
     * size is not needed.
     * @return The status of the operation. Returns `Status::OK` if the option
     * was retrieved successfully, or an appropriate error code if the operation
     * failed. You need to convert the value to host order.
     */
    Status getOption(const uint16_t &address, void *value, size_t size,
                     size_t *size_read);

    /**
     * Retrieves the value of a specific option from the lidar device.
     *
     * @tparam option The option to retrieve.
     * @return The value of the specified option.
     */
    template <LidarOption option>
    typename LidarOptionTrait<option>::type getOption() {
        using ExpectedType = typename LidarOptionTrait<option>::type;
        if constexpr (std::is_same_v<ExpectedType, int32_t> ||
                      std::is_same_v<ExpectedType, uint32_t>) {
            return getIntOption(option);
        } else if constexpr (std::is_same_v<ExpectedType, int16_t> ||
                             std::is_same_v<ExpectedType, uint16_t>) {
            return getUint16Option(option);
        } else if constexpr (std::is_same_v<ExpectedType, std::string>) {
            constexpr size_t expected_size = LidarOptionSizeTrait<option>::size;
            char value[expected_size];
            size_t size_read = 0;
            getOption(option, value, expected_size, &size_read);
            if (size_read < expected_size) {
                value[size_read] = '\0';
            } else {
                value[expected_size - 1] = '\0';
            }
            return std::string(value);
        } else if constexpr (std::is_same_v<ExpectedType, bool>) {
            return getBoolOption(option);
        } else if constexpr (std::is_same_v<ExpectedType, float>) {
            return getFloatOption(option);
        } else {
            ExpectedType data;
            getOption(option, &data, sizeof(ExpectedType));
            return data;
        }
    }

    /**
     * \brief Checks if the specified option is supported.
     *
     * \param option The option to check for support.
     * \return True if the option is supported, otherwise false.
     */
    bool isOptionSupported(const LidarOption &option);
    /**
     * \brief Checks if the specified option with the given permission is
     * supported.
     *
     * \param type The option to check for support.
     * \param permission The permission level to check for the option.
     * \return True if the option with the specified permission is supported,
     * otherwise false.
     */
    bool isOptionSupported(const LidarOption &type,
                           const LidarOptionPermission &permission);

    /**
     * \brief Checks if the specified stream type is supported.
     *
     * \param type The stream type to check for support.
     * \return True if the stream type is supported, otherwise false.
     */
    bool isStreamSupported(const LidarStreamType &type);
    /**
     * \brief Retrieves the model of the LiDAR device.
     *
     * \return The model of the LiDAR device.
     */
    LidarModel getModel();

    /**
     * \brief Retrieves the name of the LiDAR device. Must be unique name for
     * each device.
     *
     * \return The name of the LiDAR device.
     */
    std::string getName();

    /**
     * @brief Sets the name of the device.
     *
     * This function sets the name of the device to the specified value.
     *
     * @param name The name of the device, Must be unique name for each device.
     */
    void setDeviceName(const std::string &name);

   private:
    std::unique_ptr<detail::DeviceImpl> impl_;
};

}  // namespace ob_lidar
