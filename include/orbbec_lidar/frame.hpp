#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "types.hpp"
namespace ob_lidar {
class FrameSet;
class Frame;
namespace detail {
class FrameImpl;
class ImuFrameImpl;
class PointCloudFrameImpl;
class ScanFrameImpl;
}  // namespace detail

/**
 * @typedef FrameCallback
 * @brief Callback function type for handling frames.
 *
 * This type defines a callback function that takes a `std::shared_ptr<Frame>`
 * as a parameter. It is used to process frames in real-time as they are
 * received.
 *
 * @warning The callback function should not block or perform long-running
 * operations. Blocking the callback can cause delays in frame processing,
 * leading to dropped frames or degraded performance in the system. Ensure that
 * the callback returns quickly, and offload any heavy processing to a separate
 * thread or asynchronous task.
 */
using FrameCallback = std::function<void(std::shared_ptr<Frame>)>;

// null frame type

class Frame : public std::enable_shared_from_this<Frame> {
   public:
    /**
     * @brief Constructs a Frame object.
     *
     * @param impl A unique pointer to the implementation of the Frame.
     */
    explicit Frame(std::unique_ptr<detail::FrameImpl> impl);

    /**
     * @brief Copies the metadata from another frame.
     * metadata means extra information of the frame, like timestamp, frame
     * type, etc. not include data and data size. This function copies the
     * metadata from the provided frame to the current frame.
     *
     * @param other The frame from which to copy the metadata.
     */
    virtual void copyMetaFrom(const std::shared_ptr<Frame> &other);

    virtual ~Frame();

    /**
     * @brief Get the type of the lidar frame.
     *
     * @return The type of the lidar frame.
     */
    virtual LidarFrameType type();

    /**
     * @brief Returns a pointer to the data of the frame.
     *
     * @return A const pointer to the data of the frame.
     */
    [[nodiscard]] virtual const uint8_t *data() const;

    /**
     * @brief Returns a pointer to the data of the frame.
     *
     * @return A pointer to the data of the frame.
     */
    virtual uint8_t *data();

    /**
     * @brief Returns the size of the frame.
     *
     * This function returns the size of the frame in bytes.
     *
     * @return The size of the frame.
     */
    virtual size_t size();

    /**
     * @brief Get the frame ID of the lidar frame.
     *
     * @return The frame ID as a uint16_t.
     */
    virtual uint16_t frameId() const;

    /**
     * @brief Get the sync mode of the lidar frame.
     *
     * @return The sync mode as a uint8_t.
     */
    virtual uint8_t syncMode() const;

    /**
     * @brief Checks if the frame is null.
     *
     * @return true if the frame is null, false otherwise.
     */
    virtual bool isNull();

    /**
     * @brief Returns the timestamp of the frame.
     *
     * @return The timestamp of the frame in nanoseconds.
     */
    virtual std::chrono::nanoseconds timestamp();

   protected:
    Frame();
    friend class ScanFrame;
    friend class PointCloudFrame;
    const std::unique_ptr<detail::FrameImpl> impl_;
};

// NullFrame for kill nullptr
class NullFrame : public Frame {
   public:
    NullFrame();

    ~NullFrame() override;

    LidarFrameType type() override;

    [[nodiscard]] const uint8_t *data() const override;
    uint8_t *data() override;

    size_t size() override;

    bool isNull() override;

    std::chrono::nanoseconds timestamp() override;

    void copyMetaFrom(const std::shared_ptr<Frame> &other) override;

   private:
    uint8_t empty_data_[1] = {0};
};

class PointCloudFrame : public Frame {
   public:
    explicit PointCloudFrame(std::unique_ptr<detail::PointCloudFrameImpl> impl);
    ~PointCloudFrame() override;

    /**
     * @brief Retrieves the distance scaling factor.
     *
     * This function returns the scaling factor used to convert raw distance
     * data to millimeters (mm). The raw distance values should be multiplied by
     * this scaling factor to obtain the actual distance in mm.
     *
     * For example, if the scaling factor is 2, it means the raw distance unit
     * represents 2mm, so multiplying the raw value by 2 will give the distance
     * in mm.
     *
     * @return The distance scaling factor. Multiply raw distance values by this
     *         factor to get the distance in mm. A value less than or equal to 0
     *         indicates an invalid scaling factor.
     */
    double getDistanceScale() const;

    /**
     * @brief Gets the angle scaling factor of the frame.
     *
     * This function returns the angle scaling factor of the frame. The raw
     * angle values should be multiplied by this scaling factor to obtain the
     * actual angle in degrees.
     *
     * For example, if the scaling factor is 0.01, it means the raw angle unit
     * represents 0.01 degrees, so multiplying the raw value by 0.01 will give
     * the angle in degrees.
     *
     * @return The angle scaling factor. Multiply raw angle values by this
     * factor to get the angle in degrees.
     */
    double getAngleScale() const;

    /**
     * @brief Copies the metadata from another frame.
     *
     * @see Frame::copyMetaFrom
     */
    void copyMetaFrom(const std::shared_ptr<Frame> &other) override;

    /**
     * @brief Retrieves the lidar point cloud.
     *
     * This function returns the lidar point cloud associated with the frame.
     *
     * @return The lidar point cloud.
     */
    LidarPointCloud toPointCloud() const;
};

class ScanFrame : public Frame {
   public:
    ScanFrame(std::unique_ptr<detail::ScanFrameImpl> impl);
    ~ScanFrame() override;

    /**
     * @brief Retrieves the lidar scan.
     *
     * This function returns the lidar scan associated with the frame.
     *
     * @return The lidar scan.
     */
    LidarScan toScan() const;

    /**
     * @brief Copies the metadata from another frame.
     *
     * @see Frame::copyMetaFrom
     */
    void copyMetaFrom(const std::shared_ptr<Frame> &other) override;
};
}  // namespace ob_lidar
