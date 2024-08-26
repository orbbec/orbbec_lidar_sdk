#include "orb_lidar_driver/frame.hpp"

#include <memory>

#include "detail/frame.hpp"

namespace ob_lidar_driver {
Frame::Frame(std::unique_ptr<detail::FrameImpl> impl)
    : impl_(std::move(impl)) {
}

Frame::Frame()
    : impl_(nullptr) {
}

const uint8_t* Frame::data() const { return impl_->data(); }
uint8_t* Frame::data() { return impl_->data(); }

size_t Frame::size() { return impl_->size(); }

bool Frame::isNull() { return false; }

Frame::~Frame() = default;

uint16_t Frame::frameId() const {
    return impl_->frameId();
}


LidarFrameType Frame::type() { return impl_->type(); }

std::chrono::nanoseconds Frame::timestamp() { return impl_->timestamp(); }

NullFrame::NullFrame() = default;

NullFrame::~NullFrame() = default;

LidarFrameType NullFrame::type() { return LidarFrameType::UNKNOWN; }

const uint8_t* NullFrame::data() const { return empty_data_; }

uint8_t* NullFrame::data() { return empty_data_; }

size_t NullFrame::size() { return 0; }

bool NullFrame::isNull() { return true; }

std::chrono::nanoseconds NullFrame::timestamp() {
    return std::chrono::nanoseconds(0);
}

PointCloudFrame::PointCloudFrame(
    std::unique_ptr<detail::PointCloudFrameImpl> impl)
    : Frame(std::move(impl)) {
}

PointCloudFrame::~PointCloudFrame() = default;

double PointCloudFrame::getAngleScale() const {
    if (const auto impl = dynamic_cast<detail::PointCloudFrameImpl*>(impl_.
        get())) {
        return impl->getAngleScale();
    }
    return 0.0;
}

double PointCloudFrame::getDistanceScale() const {
    if (const auto impl = dynamic_cast<detail::PointCloudFrameImpl*>(impl_.
        get())) {
        return impl->getDistanceScale();
    }
    return 0.0;
}
} // namespace ob_lidar_driver
