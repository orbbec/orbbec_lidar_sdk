#pragma once

#include <memory>
#include <vector>

#include "orbbec_lidar/filter.hpp"

namespace ob_lidar::detail {

class OutlierRemovalFilterImpl {
   public:
    OutlierRemovalFilterImpl(int level, int scan_speed);

    ~OutlierRemovalFilterImpl();

    void setFilterWindowSize(int window_size);

    [[nodiscard]] std::shared_ptr<Frame> process(
        std::shared_ptr<Frame> frame) const;

   private:
    void filter(std::vector<uint16_t>& ranges) const;

    int level_;
    int scan_speed_;
    int window_size_ = 7;  // NOTE: must be odd number
};

}  // namespace ob_lidar::detail
