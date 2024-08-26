#pragma once

#include "capability.hpp"

namespace ob_lidar {
class MS600LidarCapabilities : public LidarCapabilitiesInterface {
   public:
    MS600LidarCapabilities();

    ~MS600LidarCapabilities() override = default;

    bool isOptionSupported(const LidarOption &option) const override;

    bool isOptionSupported(
        const LidarOption &type,
        const LidarOptionPermission &permission) const override;

    bool isStreamSupported(const LidarStreamType &type) const override;

    bool isStreamFreqSupported(const LidarStreamType &type,
                               const double &freq) const override;
};
}  // namespace ob_lidar
