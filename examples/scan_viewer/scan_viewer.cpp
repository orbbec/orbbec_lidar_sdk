#include <pcl/visualization/pcl_visualizer.h>

#include <condition_variable>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>

#include "orbbec_lidar/orbbec_lidar.hpp"

std::queue<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_queue;
std::mutex cloud_mutex;
std::condition_variable cloud_cv;
namespace ob = ob_lidar;

double deg2rad(double degree) { return degree * M_PI / 180.0; }
ob::LidarPointCloud scanToPointCloud(const ob::LidarScan& scan) {
    ob::LidarPointCloud point_cloud;
    point_cloud.points.reserve(scan.ranges.size());
    auto start_angle = scan.start_angle;
    auto end_angle = scan.end_angle;
    auto angle_resolution = scan.angle_resolution;
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        ob::LidarPoint point{};
        const auto angle = start_angle + i * angle_resolution;

        double rad = deg2rad(angle);
        point.x = scan.ranges[i] * cos(rad);
        point.y = scan.ranges[i] * sin(rad);
        point.z = 0;
        point.reflectivity = scan.intensities[i];
        point_cloud.points.push_back(point);
    }
    return point_cloud;
}

void frameCallback(const std::shared_ptr<ob::Frame>& frame) {
    if (frame == nullptr) {
        std::cerr << "frame is nullptr" << std::endl;
    }
    if (frame->type() == ob::LidarFrameType::SCAN) {
        auto scan_frame = std::dynamic_pointer_cast<ob::ScanFrame>(frame);
        if (scan_frame == nullptr) {
            std::cerr << "point_cloud is nullptr" << std::endl;
        }
        auto scan = scan_frame->toScan();

        // Convert custom point cloud to PCL point cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(
            new pcl::PointCloud<pcl::PointXYZI>);
        pcl_cloud->reserve(scan.ranges.size());  // Reserve space for efficiency

        auto point_cloud = scanToPointCloud(scan);
        for (size_t i = 0; i < point_cloud.points.size(); ++i) {
            pcl::PointXYZI pcl_point;
            pcl_point.x = point_cloud.points[i].x / 1000.0f;
            pcl_point.y = point_cloud.points[i].y / 1000.0f;
            pcl_point.z = point_cloud.points[i].z / 1000.0f;
            pcl_point.intensity = point_cloud.points[i].reflectivity;
            pcl_cloud->push_back(pcl_point);
        }

        // Add PCL point cloud to queue
        {
            std::unique_lock lock(cloud_mutex);
            cloud_queue.push(pcl_cloud);
        }
        cloud_cv.notify_one();  // Notify the visualization thread
    }
}
int main() {
    ob::DeviceConfigBuilder builder;
    char buf[PATH_MAX];
    char* ptr = realpath(__FILE__, buf);
    std::string current_path = std::filesystem::path(ptr).parent_path();
    const std::string config_file_path = current_path + "/ms600_config.toml";
    auto device_config = std::make_shared<ob::DeviceConfig>(config_file_path);
    auto device_manager = std::make_shared<ob::DeviceManager>();
    device_manager->addDevice(device_config);
    auto device = device_manager->getDevice(device_config->getDeviceName());
    auto stream_config = std::make_shared<ob::StreamConfig>();
    stream_config->enableStream(ob::LidarStreamType::SCAN, 30);
    device->start(stream_config, frameCallback);

    pcl::visualization::PCLVisualizer viewer("Point Cloud Stream");
    viewer.setBackgroundColor(1.0, 1.0, 1.0);
    while (!viewer.wasStopped()) {
        std::unique_lock lock(cloud_mutex);
        cloud_cv.wait(lock, [] { return !cloud_queue.empty(); });
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = cloud_queue.front();
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>
            red_color(cloud, 255, 0, 0);
        cloud_queue.pop();
        viewer.removeAllPointClouds();
        viewer.addPointCloud<pcl::PointXYZI>(cloud, red_color, "cloud");
        viewer.setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
        viewer.spinOnce();
    }
}
