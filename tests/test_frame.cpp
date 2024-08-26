#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <nlohmann/json.hpp>
#include <toml++/toml.hpp>

#include "detail/frame.hpp"
#include "orbbec_lidar/orbbec_lidar.hpp"

TEST(Frame, test_point_cloud_frame) {
    auto frame_impl =
        std::make_unique<ob_lidar::detail::PointCloudFrameImpl>();
    EXPECT_NE(frame_impl, nullptr);
    ob_lidar::detail::PointCloudMeta meta;
    meta.angle_scale = 0.02;
    meta.distance_scale = 1.0;
    frame_impl->setMeta(meta);
    frame_impl->setFrameId(42);
    frame_impl->setTimestamp(std::chrono::nanoseconds(1000000));
    frame_impl->setFrameType(ob_lidar::LidarFrameType::POINT_CLOUD);
    std::vector<uint8_t> fake_points;
    ob_lidar::LidarPoint fake_point{};
    fake_point.x = 1;
    fake_point.y = 2;
    fake_point.z = 3;
    fake_point.reflectivity = 4;
    fake_point.tag = 5;
    // 5 points
    for (int i = 0; i < 5; i++) {
        fake_points.insert(
            fake_points.end(), reinterpret_cast<uint8_t*>(&fake_point),
            reinterpret_cast<uint8_t*>(&fake_point) + sizeof(fake_point));
    }
    frame_impl->setData(fake_points.data(), fake_points.size());
    auto frame = std::make_unique<ob_lidar::PointCloudFrame>(
        std::move(frame_impl));
    EXPECT_NE(frame, nullptr);
    EXPECT_FLOAT_EQ(frame->getAngleScale(), 0.02);
    EXPECT_FLOAT_EQ(frame->getDistanceScale(), 1.0);
    EXPECT_EQ(frame->frameId(), 42);
    EXPECT_EQ(frame->size(), 5 * sizeof(ob_lidar::LidarPoint));
    EXPECT_EQ(frame->type(), ob_lidar::LidarFrameType::POINT_CLOUD);
    EXPECT_EQ(frame->timestamp(), std::chrono::nanoseconds(1000000));
    // cast data to LidarPoint
    auto points =
        reinterpret_cast<const ob_lidar::LidarPoint*>(frame->data());
    // get the first point
    EXPECT_EQ(points[0].x, 1);
    EXPECT_EQ(points[0].y, 2);
    EXPECT_EQ(points[0].z, 3);
    EXPECT_EQ(points[0].reflectivity, 4);
    EXPECT_EQ(points[0].tag, 5);
}
