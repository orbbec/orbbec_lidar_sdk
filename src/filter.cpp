#include "orbbec_lidar/filter.hpp"

#include "detail/filter.hpp"

namespace ob_lidar {

OutlierRemovalFilter::OutlierRemovalFilter(int level, int scan_speed)
    : impl_(std::make_unique<detail::OutlierRemovalFilterImpl>(level,
                                                               scan_speed)) {}

std::shared_ptr<Frame> OutlierRemovalFilter::process(
    std::shared_ptr<Frame> frame) {
    return impl_->process(frame);
}


OutlierRemovalFilter::~OutlierRemovalFilter() = default;

void OutlierRemovalFilter::setFilterWindowSize(int window_size) {
    impl_->setFilterWindowSize(window_size);
}

}  // namespace ob_lidar
