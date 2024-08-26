#pragma once
#include "frame.hpp"
namespace ob_lidar {

namespace detail {
class OutlierRemovalFilterImpl;
}  // namespace detail

class Filter {
   public:
    Filter() = default;
    virtual ~Filter() = default;
    virtual std::shared_ptr<Frame> process(std::shared_ptr<Frame> frame) = 0;
};

// laser scan outlier removal filter
class OutlierRemovalFilter : public Filter {
   public:
    OutlierRemovalFilter(int level, int scan_speed);

    void setFilterWindowSize(int window_size);

    ~OutlierRemovalFilter() override;

    std::shared_ptr<Frame> process(std::shared_ptr<Frame> frame) override;

   private:
    std::unique_ptr<detail::OutlierRemovalFilterImpl> impl_;
};
}  // namespace ob_lidar
