#include <csignal>  // For signal handling
#include <iostream>
#include <thread>

// Signal handler
volatile sig_atomic_t g_interrupted = 0;  // Flag to indicate interruption

void signal_handler(int signum) {
    if (signum == SIGINT) {
        g_interrupted = 1;
    }
}

#include "orb_lidar_driver/driver.hpp"

namespace ob = ob_lidar_driver;

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
        for (size_t i = 0; i < 10 && i < point_size; ++i) {
            std::cout << "x: " << points[i].x << " y: " << points[i].y
                      << " z: " << points[i].z
                      << "reflectivity: " << points[i].reflectivity
                      << std::endl;
        }
    }
}

int main() {
    signal(SIGINT, signal_handler);
    auto driver = std::make_shared<ob::Driver>();
    ob::DeviceConfigBuilder builder;
    builder.setDeviceName("orb_lidar")
        .setModel("TL2401")
        .setProtocolType("TCP");
    auto device_config = builder.build();
    driver->addDevice(device_config);
    auto device = driver->getDevice(device_config->getDeviceName());
    auto stream_config = std::make_shared<ob::StreamConfig>();
    stream_config->enableStream(ob::LidarStreamType::POINT_CLOUD, 200);
    device->start(stream_config, frameCallback);
    // ctrl + c signal handling
    while (!g_interrupted) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    device->stop();
    return 0;
}
