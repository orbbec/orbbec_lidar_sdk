#pragma once

#include <gtest/gtest.h>

#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <uvw.hpp>
#include <vector>

#include "tcp_server.hpp"
#include "udp_server.hpp"

namespace ob_test {

class FakeTCPSever : public ::testing::Test {
   protected:
    void SetUp() override {
        loop_ = uvw::loop::create();
        server_1_ =
            std::make_shared<ob_test::TcpServer>(loop_, "0.0.0.0", 2401);
        server_2_ =
            std::make_shared<ob_test::TcpServer>(loop_, "0.0.0.0", 2402);
        loop_thread_ =
            std::make_shared<std::thread>([this]() { loop_->run(); });
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    void TearDown() override {
        if (loop_thread_ && loop_thread_->joinable()) {
            loop_->walk([](auto &&handle) { handle.close(); });
            loop_->close();
            loop_thread_->detach();
        }
    }

    std::shared_ptr<ob_test::TcpServer> server_1_;  // TCP server
    std::shared_ptr<ob_test::TcpServer> server_2_;  // TCP server
    std::shared_ptr<uvw::loop> loop_;
    std::shared_ptr<std::thread> loop_thread_;
};

class FakeUDPServer : public ::testing::Test {
   protected:
    void SetUp() override {
        loop_ = uvw::loop::create();
        server_1_ =
            std::make_shared<ob_test::UdpServer>(loop_, "0.0.0.0", 2401);
        server_2_ =
            std::make_shared<ob_test::UdpServer>(loop_, "0.0.0.0", 2402);
        loop_thread_ =
            std::make_shared<std::thread>([this]() { loop_->run(); });
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    void TearDown() override {
        if (loop_thread_ && loop_thread_->joinable()) {
            loop_->walk([](auto &&handle) { handle.close(); });
            loop_->close();
            loop_thread_->detach();
        }
    }

    std::shared_ptr<ob_test::UdpServer> server_1_;
    std::shared_ptr<ob_test::UdpServer> server_2_;
    std::shared_ptr<uvw::loop> loop_;
    std::shared_ptr<std::thread> loop_thread_;
};

}  // namespace ob_test

namespace ob_test {

inline int ipAddressToInt(const std::string &ip) {
    int result = 0;
    int a, b, c, d;
    sscanf(ip.c_str(), "%d.%d.%d.%d", &a, &b, &c, &d);
    result = (a << 24) | (b << 16) | (c << 8) | d;
    return result;
}

inline std::string ipAddressToString(int ip) {
    return std::to_string((ip >> 24) & 0xFF) + "." +
           std::to_string((ip >> 16) & 0xFF) + "." +
           std::to_string((ip >> 8) & 0xFF) + "." + std::to_string(ip & 0xFF);
}

inline std::vector<std::string> splitString(const std::string &str,
                                            char delimiter) {
    std::vector<std::string> result;
    std::stringstream ss(str);
    std::string item;
    while (std::getline(ss, item, delimiter)) {
        result.push_back(item);
    }
    return result;
}

inline std::string macAddressToString(const std::vector<uint8_t> &mac) {
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (size_t i = 0; i < mac.size(); ++i) {
        ss << std::setw(2) << static_cast<int>(mac[i]);
        if (i < mac.size() - 1) {
            ss << ":";
        }
    }
    return ss.str();
}

inline std::vector<uint8_t> macAddressToBytes(const std::string &mac) {
    std::vector<uint8_t> result;
    std::stringstream ss(mac);
    std::string item;
    while (std::getline(ss, item, ':')) {
        result.push_back(std::stoi(item, nullptr, 16));
    }
    return result;
}

}  // namespace ob_test
