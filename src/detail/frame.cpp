#include "frame.hpp"

#include <cstring>

namespace ob_lidar_driver::detail {
FrameImpl::FrameImpl() = default;

void FrameImpl::setData(const uint8_t* data, size_t size) {
    data_ = new uint8_t[size];
    size_ = size;
    memcpy(data_, data, size);
}

void FrameImpl::setFrameId(uint16_t frame_id) { frame_id_ = frame_id; }

void FrameImpl::setFrameType(LidarFrameType type) { type_ = type; }

void FrameImpl::setTimestamp(std::chrono::nanoseconds timestamp) {
    timestamp_ = timestamp;
}

FrameImpl::~FrameImpl() { delete[] data_; }

uint8_t* FrameImpl::data() { return data_; }

const uint8_t* FrameImpl::data() const { return data_; }

LidarFrameType FrameImpl::type() const { return type_; }

size_t FrameImpl::size() const { return size_; }

std::chrono::nanoseconds FrameImpl::timestamp() const { return timestamp_; }

uint16_t FrameImpl::frameId() const {
    return frame_id_;
}

PointCloudFrameImpl::PointCloudFrameImpl() = default;

void PointCloudFrameImpl::setDistanceScale(double scale) {
    distance_scale_ = scale;
}

double PointCloudFrameImpl::getDistanceScale() const { return distance_scale_; }

void PointCloudFrameImpl::setAngleScale(double scale) { angle_scale_ = scale; }

double PointCloudFrameImpl::getAngleScale() const { return angle_scale_; }

PointCloudFrameImpl::~PointCloudFrameImpl() = default;
} // namespace ob_lidar_driver::detail
