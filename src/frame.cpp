#include "orbbec_lidar/frame.hpp"

#include <memory>

#include "detail/frame.hpp"

namespace ob_lidar {
Frame::Frame(std::unique_ptr<detail::FrameImpl> impl)
    : impl_(std::move(impl)) {}

Frame::Frame() : impl_(nullptr) {}

const uint8_t* Frame::data() const { return impl_->data(); }
uint8_t* Frame::data() { return impl_->data(); }

size_t Frame::size() { return impl_->size(); }

bool Frame::isNull() { return false; }

Frame::~Frame() = default;
void Frame::copyMetaFrom(const std::shared_ptr<Frame>& other) {
    impl_->copyMetaFrom(other->impl_.get());
}
uint16_t Frame::frameId() const { return impl_->frameId(); }

uint8_t Frame::syncMode() const { return impl_->syncMode(); }

LidarFrameType Frame::type() { return impl_->type(); }

std::chrono::nanoseconds Frame::timestamp() { return impl_->timestamp(); }

NullFrame::NullFrame() = default;

NullFrame::~NullFrame() = default;

LidarFrameType NullFrame::type() { return LidarFrameType::UNKNOWN; }

const uint8_t* NullFrame::data() const { return empty_data_; }

uint8_t* NullFrame::data() { return empty_data_; }

size_t NullFrame::size() { return 0; }

bool NullFrame::isNull() { return true; }

void NullFrame::copyMetaFrom(const std::shared_ptr<Frame>& other) {
    (void)other;
}

std::chrono::nanoseconds NullFrame::timestamp() {
    return std::chrono::nanoseconds(0);
}

PointCloudFrame::PointCloudFrame(
    std::unique_ptr<detail::PointCloudFrameImpl> impl)
    : Frame(std::move(impl)) {}

PointCloudFrame::~PointCloudFrame() = default;

void PointCloudFrame::copyMetaFrom(const std::shared_ptr<Frame>& other) {
    if (const auto impl =
            dynamic_cast<detail::PointCloudFrameImpl*>(impl_.get())) {
        if (const auto other_impl = dynamic_cast<detail::PointCloudFrameImpl*>(
                other->impl_.get())) {
            impl->copyMetaFrom(other_impl);
        }
    }
}

double PointCloudFrame::getAngleScale() const {
    if (const auto impl =
            dynamic_cast<detail::PointCloudFrameImpl*>(impl_.get())) {
        return impl->getAngleScale();
    }
    return 0.0;
}

double PointCloudFrame::getDistanceScale() const {
    if (const auto impl =
            dynamic_cast<detail::PointCloudFrameImpl*>(impl_.get())) {
        return impl->getDistanceScale();
    }
    return 0.0;
}

LidarPointCloud PointCloudFrame::toPointCloud() const {
    if (const auto impl =
            dynamic_cast<detail::PointCloudFrameImpl*>(impl_.get())) {
        return impl->toPointCloud();
    }
    return {};
}

ScanFrame::ScanFrame(std::unique_ptr<detail::ScanFrameImpl> impl)
    : Frame(std::move(impl)) {}

ScanFrame::~ScanFrame() = default;

LidarScan ScanFrame::toScan() const {
    if (const auto impl = dynamic_cast<detail::ScanFrameImpl*>(impl_.get())) {
        return impl->toScan();
    }
    return {};
}

void ScanFrame::copyMetaFrom(const std::shared_ptr<Frame>& other) {
    if (const auto impl = dynamic_cast<detail::ScanFrameImpl*>(impl_.get())) {
        if (const auto other_impl =
                dynamic_cast<detail::ScanFrameImpl*>(other->impl_.get())) {
            impl->copyMetaFrom(other_impl);
        }
    }
}
}  // namespace ob_lidar
