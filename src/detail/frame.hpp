#pragma once

#include <chrono>
#include <map>
#include <memory>

#include "orb_lidar_driver/frame.hpp"
namespace ob_lidar_driver::detail {

class FrameImpl {
   public:
    FrameImpl();

    void setData(const uint8_t *data, size_t size);

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

   protected:
    LidarFrameType type_ = LidarFrameType::UNKNOWN;
    size_t size_ = 0;
    uint8_t *data_ = nullptr;
    std::chrono::nanoseconds timestamp_ = std::chrono::nanoseconds(0);
    uint16_t frame_id_ = 0;
};

class PointCloudFrameImpl : public FrameImpl {
   public:
    PointCloudFrameImpl();

    ~PointCloudFrameImpl() override;

    void setDistanceScale(double scale);

    void setAngleScale(double scale);

    [[nodiscard]] double getDistanceScale() const;

    [[nodiscard]] double getAngleScale() const;

   private:
    double distance_scale_ = 2.0;
    double angle_scale_ = 0.01;
};

}  // namespace ob_lidar_driver::detail
