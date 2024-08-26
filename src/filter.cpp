#include "orb_lidar_driver/filter.hpp"

namespace ob_lidar_driver {
SmoothFilter::SmoothFilter(float radius) : radius_(radius) {}

void SmoothFilter::process(std::shared_ptr<Frame> frame) {
    // TODO: implement smooth filter
}

}  // namespace ob_lidar_driver
