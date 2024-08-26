#pragma once

#include <any>
#include <future>
#include <iostream>
#include <magic_enum/magic_enum.hpp>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <uvw.hpp>
#include <vector>

#include "orb_lidar_driver/config.hpp"
#include "orb_lidar_driver/types.hpp"
#include "types.hpp"

namespace ob_lidar_driver::detail {

using OnDataCallback = std::function<void(const std::vector<uint8_t>&)>;

// Channel abstraction
class Channel {
   public:
    virtual ~Channel() = default;
    virtual void send(const std::vector<uint8_t>& buffer) = 0;
    virtual void setOnDataCallback(const OnDataCallback& callback) = 0;

   protected:
    OnDataCallback callback_;
    uvw::socket_address socket_address_{};
};

// TCP Channel implementation
class TCPChannel final : public Channel {
   public:
    TCPChannel(std::shared_ptr<uvw::loop> loop, const std::string& ip,
               uint32_t port);

    ~TCPChannel() override;
    void send(const std::vector<uint8_t>& buffer) override;
    void setOnDataCallback(const OnDataCallback& callback) override;

   private:
    std::shared_ptr<uvw::tcp_handle> handle_ = nullptr;
};

// UDP Channel implementation
class UDPChannel final : public Channel {
   public:
    UDPChannel(std::shared_ptr<uvw::loop> loop, const std::string& ip,
               uint32_t port);
    ~UDPChannel() override;
    void send(const std::vector<uint8_t>& buffer) override;
    void setOnDataCallback(const OnDataCallback& callback) override;

   private:
    std::shared_ptr<uvw::udp_handle> handle_ = nullptr;
};

// NetworkComm using Strategy pattern
class NetworkComm {
   public:
    explicit NetworkComm(LidarProtocolType protocol);

 ~NetworkComm();

    void createChannel(LidarChannelType type, const std::string& ip,
                       uint32_t port);
    void removeChannel(LidarChannelType type);

    void sendData(LidarChannelType type, const std::vector<uint8_t>& buffer);

    void setOnDataCallback(LidarChannelType type,
                           const OnDataCallback& callback);

    // For single channel mode
    void createSingleChannel(const std::string& ip, uint32_t port);

    void sendData(const std::vector<uint8_t>& buffer) const;

    void setOnDataCallback(const OnDataCallback& callback) const;

   void runLoop();

   private:
    LidarProtocolType protocol_;
    std::shared_ptr<uvw::loop> loop_;
    std::map<LidarChannelType, std::unique_ptr<Channel>> channels_;
    std::unique_ptr<Channel> single_channel_;
    bool is_single_channel_mode_ = false;
    std::shared_ptr<std::thread> loop_thread_;
};

// Factory for creating NetworkComm
class NetworkCommFactory {
   public:
    static std::unique_ptr<NetworkComm> create(
        const std::shared_ptr<NetworkConfig>& config);
};

}  // namespace ob_lidar_driver::detail
