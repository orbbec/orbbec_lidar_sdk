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

void frameCallback(const std::shared_ptr<ob::Frame>& frame) {
    if (frame == nullptr) {
        std::cerr << "frame is nullptr" << std::endl;
    }
    if (frame->type() == ob::LidarFrameType::POINT_CLOUD) {
        auto point_cloud =
            std::dynamic_pointer_cast<ob::PointCloudFrame>(frame);
        if (point_cloud == nullptr) {
            std::cerr << "point_cloud is nullptr" << std::endl;
        }
        const auto& data = frame->data();
        auto data_size = frame->size();
        size_t point_size = data_size / sizeof(ob::LidarPoint);
        auto points = reinterpret_cast<const ob::LidarPoint*>(data);

        // Convert custom point cloud to PCL point cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(
            new pcl::PointCloud<pcl::PointXYZI>);
        pcl_cloud->reserve(point_size);  // Reserve space for efficiency

        double distance_scale = point_cloud->getDistanceScale();

        for (size_t i = 0; i < point_size; ++i) {
            pcl::PointXYZI pcl_point;
            pcl_point.x = points[i].x * distance_scale;
            pcl_point.y = points[i].y * distance_scale;
            pcl_point.z = points[i].z * distance_scale;
            pcl_point.intensity =
                points[i].reflectivity;  // Assuming reflectivity is used as
                                         // intensity
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
    std::string parent_path =
        std::filesystem::path(ptr).parent_path().parent_path();
    const std::string config_file_path =
        parent_path + "/config/single_device_config.toml";
    auto device_config = std::make_shared<ob::DeviceConfig>(config_file_path);
    auto device_manager = std::make_shared<ob::DeviceManager>();
    device_manager->addDevice(device_config);
    auto device = device_manager->getDevice(device_config->getDeviceName());
    auto stream_config = std::make_shared<ob::StreamConfig>();
    stream_config->enableStream(ob::LidarStreamType::POINT_CLOUD, 200);
    device->start(stream_config, frameCallback);

    pcl::visualization::PCLVisualizer viewer("Point Cloud Stream");
    while (!viewer.wasStopped()) {
        std::unique_lock lock(cloud_mutex);
        cloud_cv.wait(lock, [] { return !cloud_queue.empty(); });
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = cloud_queue.front();
        cloud_queue.pop();
        viewer.removeAllPointClouds();
        viewer.addPointCloud<pcl::PointXYZI>(cloud, "cloud");
        viewer.spinOnce();
    }
}
