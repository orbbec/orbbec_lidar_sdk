#include "frame.hpp"

#include <netinet/in.h>

#include <check.hpp>
#include <cstring>

namespace ob_lidar::detail {
FrameImpl::FrameImpl() = default;

FrameImpl::FrameImpl(size_t size) : size_(size) {
    data_ = new uint8_t[size];
    memset(data_, 0, size);
}

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

void FrameImpl::setSyncMode(uint8_t sync_mode) { sync_mode_ = sync_mode; }

void FrameImpl::copyMetaFrom(const FrameImpl* other) {
    if (other == nullptr) {
        return;
    }
    timestamp_ = other->timestamp_;
    frame_id_ = other->frame_id_;
    type_ = other->type_;
}

FrameImpl::~FrameImpl() { delete[] data_; }

uint8_t* FrameImpl::data() { return data_; }

const uint8_t* FrameImpl::data() const { return data_; }

LidarFrameType FrameImpl::type() const { return type_; }

size_t FrameImpl::size() const { return size_; }

std::chrono::nanoseconds FrameImpl::timestamp() const { return timestamp_; }

uint16_t FrameImpl::frameId() const { return frame_id_; }

uint8_t FrameImpl::syncMode() const { return sync_mode_; }

PointCloudFrameImpl::PointCloudFrameImpl() = default;

PointCloudFrameImpl::PointCloudFrameImpl(size_t size) : FrameImpl(size) {}

PointCloudFrameImpl::~PointCloudFrameImpl() = default;

void PointCloudFrameImpl::setMeta(const PointCloudMeta& meta) {
    distance_scale_ = meta.distance_scale;
    angle_scale_ = meta.angle_scale;
}

double PointCloudFrameImpl::getDistanceScale() const { return distance_scale_; }

double PointCloudFrameImpl::getAngleScale() const { return angle_scale_; }

void PointCloudFrameImpl::copyMetaFrom(const FrameImpl* other) {
    if (other == nullptr) {
        return;
    }
    FrameImpl::copyMetaFrom(other);
    auto other_impl = dynamic_cast<const PointCloudFrameImpl*>(other);
    CHECK_NOTNULL(other_impl);
    distance_scale_ = other_impl->distance_scale_;
    angle_scale_ = other_impl->angle_scale_;
}

LidarPointCloud PointCloudFrameImpl::toPointCloud() const {
    LidarPointCloud point_cloud{};
    if (size_ % sizeof(LidarPoint) != 0) {
        return point_cloud;
    }
    if (size_ == 0) {
        return point_cloud;
    }
    const auto* points = reinterpret_cast<LidarPoint*>(data_);
    CHECK_NOTNULL(points);
    size_t point_count = size_ / sizeof(LidarPointCloud);
    point_cloud.timestamp =
        std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp_)
            .count();
    point_cloud.points.reserve(point_count);
    for (size_t i = 0; i < point_count; ++i) {
        point_cloud.points.emplace_back(points[i]);
    }
    return point_cloud;
}

ScanFrameImpl::ScanFrameImpl() = default;

ScanFrameImpl::ScanFrameImpl(size_t size) : FrameImpl(size) {
    CHECK_EQ(size % 4, 0);
    scan_size_ = size / 4;
}

void ScanFrameImpl::setMeta(const LaserScanMeta& meta) {
    scan_size_ = meta.scan_size;
    start_angle_ = meta.start_angle;
    end_angle_ = meta.end_angle;
    angle_resolution_ = meta.angle_resolution;
}

void ScanFrameImpl::setRanges(const std::vector<uint16_t>& ranges) {
    CHECK_EQ(size_, scan_size_ * sizeof(uint16_t) * 2);
    CHECK_EQ(ranges.size(), scan_size_);
    memcpy(data_, ranges.data(), scan_size_ * sizeof(uint16_t));
}

void ScanFrameImpl::setIntensities(const std::vector<uint16_t>& intensities) {
    CHECK_EQ(size_, scan_size_ * sizeof(uint16_t) * 2);
    memcpy(data_ + scan_size_ * sizeof(uint16_t), intensities.data(),
           scan_size_ * sizeof(uint16_t));
}

size_t ScanFrameImpl::getScanSize() const { return scan_size_; }

double ScanFrameImpl::getStartAngle() const { return start_angle_; }

double ScanFrameImpl::getEndAngle() const { return end_angle_; }

ScanFrameImpl::~ScanFrameImpl() = default;

double ScanFrameImpl::getAngleResolution() const { return angle_resolution_; }

double ScanFrameImpl::getContaminatedAngle() const {
    return contaminated_angle_;
}

uint8_t ScanFrameImpl::getContaminatedLevel() const {
    return contaminated_level_;
}

void ScanFrameImpl::copyMetaFrom(const FrameImpl* other) {
    if (other == nullptr) {
        return;
    }
    FrameImpl::copyMetaFrom(other);
    const auto other_impl = dynamic_cast<const ScanFrameImpl*>(other);
    CHECK_NOTNULL(other_impl);
    scan_size_ = other_impl->scan_size_;
    start_angle_ = other_impl->start_angle_;
    end_angle_ = other_impl->end_angle_;
    angle_resolution_ = other_impl->angle_resolution_;
    contaminated_angle_ = other_impl->contaminated_angle_;
    contaminated_level_ = other_impl->contaminated_level_;
}

LidarScan ScanFrameImpl::toScan() const {
    LidarScan scan;
    scan.angle_resolution = angle_resolution_;
    scan.start_angle = start_angle_;
    scan.end_angle = end_angle_;
    scan.timestamp =
        std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp_)
            .count();
    scan.contaminated_level = contaminated_level_;
    scan.contaminated_angle = contaminated_angle_;
    scan.ranges.resize(scan_size_);
    scan.intensities.resize(scan_size_);
    // fist half is ranges, second half is intensities
    memcpy(scan.ranges.data(), data_, scan_size_ * sizeof(uint16_t));
    memcpy(scan.intensities.data(), data_ + scan_size_ * sizeof(uint16_t),
           scan_size_ * sizeof(uint16_t));
    return scan;
}

}  // namespace ob_lidar::detail
