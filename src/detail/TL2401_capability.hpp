#pragma once

#include "capability.hpp"

namespace ob_lidar_driver {
class TL2401LidarCapabilities : public LidarCapabilitiesInterface {
   public:
    TL2401LidarCapabilities();

    ~TL2401LidarCapabilities() override = default;

    bool isOptionSupported(const LidarOption &option) const override;

    bool isOptionSupported(
        const LidarOption &type,
        const LidarOptionPermission &permission) const override;

    bool isStreamSupported(const LidarStreamType &type) const override;

    bool isStreamFreqSupported(const LidarStreamType &type,
                               const double &freq) const override;
};
}  // namespace ob_lidar_driver
