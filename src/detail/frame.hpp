#pragma once

#include <chrono>
#include <map>
#include <memory>

#include "orbbec_lidar/frame.hpp"
#include "orbbec_lidar/types.hpp"

namespace ob_lidar::detail {

struct PointCloudMeta {
    double distance_scale = 0.0;
    double angle_scale = 0.0;
};

struct LaserScanMeta {
    uint8_t sync_mode = 0;
    size_t scan_size = 0;
    double start_angle = 0.0;
    double end_angle = 0.0;
    double angle_resolution = 0.0;

    double contaminated_angle = 0.0;
    uint8_t contaminated_level = 0;
};

class FrameImpl {
   public:
    FrameImpl();

    explicit FrameImpl(size_t size);

    void setData(const uint8_t *data, size_t size);

    virtual void copyMetaFrom(const FrameImpl *other);

    void setFrameId(uint16_t frame_id);

    void setFrameType(LidarFrameType type);

    void setTimestamp(std::chrono::nanoseconds timestamp);

    virtual ~FrameImpl();

    [[nodiscard]] LidarFrameType type() const;

    [[nodiscard]] const uint8_t *data() const;

    uint8_t *data();

    [[nodiscard]] size_t size() const;

    [[nodiscard]] std::chrono::nanoseconds timestamp() const;

    uint16_t frameId() const;

    uint8_t syncMode() const;

    void setSyncMode(uint8_t sync_mode);

   protected:
    LidarFrameType type_ = LidarFrameType::UNKNOWN;
    size_t size_ = 0;
    uint8_t *data_ = nullptr;
    std::chrono::nanoseconds timestamp_ = std::chrono::nanoseconds(0);
    uint16_t frame_id_ = 0;
    uint8_t sync_mode_ = 0;
};

class PointCloudFrameImpl : public FrameImpl {
   public:
    PointCloudFrameImpl();

    explicit PointCloudFrameImpl(size_t size);

    void copyMetaFrom(const FrameImpl *other) override;

    ~PointCloudFrameImpl() override;

    void setMeta(const PointCloudMeta &meta);

    [[nodiscard]] double getDistanceScale() const;

    [[nodiscard]] double getAngleScale() const;

    [[nodiscard]] LidarPointCloud toPointCloud() const;

   private:
    double distance_scale_ = 0.0;
    double angle_scale_ = 0.0;
};

class ScanFrameImpl : public FrameImpl {
   public:
    ScanFrameImpl();

    explicit ScanFrameImpl(size_t size);

    ~ScanFrameImpl() override;

    void setMeta(const LaserScanMeta &meta);

    void copyMetaFrom(const FrameImpl *other) override;

    [[nodiscard]] LidarScan toScan() const;

    size_t getScanSize() const;

    void setRanges(const std::vector<uint16_t> &ranges);

    void setIntensities(const std::vector<uint16_t> &intensities);

    [[nodiscard]] double getStartAngle() const;

    [[nodiscard]] double getEndAngle() const;

    [[nodiscard]] double getAngleResolution() const;

    [[nodiscard]] double getContaminatedAngle() const;

    [[nodiscard]] uint8_t getContaminatedLevel() const;

   private:
    size_t scan_size_ = 0;
    double start_angle_ = 0.0;
    double end_angle_ = 0.0;
    double angle_resolution_ = 0.0;
    double contaminated_angle_ = 0.0;
    uint8_t contaminated_level_ = 0;
};
}  // namespace ob_lidar::detail
