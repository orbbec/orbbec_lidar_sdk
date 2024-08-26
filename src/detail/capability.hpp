#pragma once

#include <memory>
#include <orbbec_lidar/option.hpp>
#include <string>
#include <unordered_map>

namespace ob_lidar {

#define REGISTER_OPTION_PERMISSION(option, permission) \
    do {                                               \
        supported_options_map_[option] = permission;   \
    } while (0)

#define REGISTER_STREAM(stream, supported)         \
    do {                                           \
        supported_stream_map_[stream] = supported; \
    } while (0)

class LidarCapabilitiesInterface {
   public:
    LidarCapabilitiesInterface() = default;

    virtual ~LidarCapabilitiesInterface() = default;

    virtual bool isOptionSupported(const LidarOption &option) const = 0;

    virtual bool isOptionSupported(
        const LidarOption &type,
        const LidarOptionPermission &permission) const = 0;

    virtual bool isStreamSupported(const LidarStreamType &type) const = 0;

    virtual bool isStreamFreqSupported(const LidarStreamType &type,
                                       const double &freq) const = 0;

   protected:
    std::unordered_map<LidarOption, LidarOptionPermission>
        supported_options_map_;
    std::unordered_map<LidarStreamType, bool> supported_stream_map_;
    std::unordered_map<LidarStreamType, std::vector<double>>
        supported_stream_freq_map_;
};
}  // namespace ob_lidar
