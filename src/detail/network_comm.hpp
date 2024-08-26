#pragma once

#include <any>
#include <foonathan/memory/container.hpp>    // vector, list, list_node_size
#include <foonathan/memory/memory_pool.hpp>  // memory_pool
#include <foonathan/memory/smart_ptr.hpp>    // allocate_unique
#include <foonathan/memory/static_allocator.hpp>  // static_allocator_storage, static_block_allocator
#include <foonathan/memory/temporary_allocator.hpp>  // temporary_allocator
#include <future>
#include <iostream>
#include <magic_enum/magic_enum.hpp>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <uvw.hpp>
#include <vector>

// alias namespace foonathan::memory as memory for easier access
#include <foonathan/memory/tracking.hpp>  // make_tracked_allocator

#include "../logger.hpp"
#include "orbbec_lidar/config.hpp"
#include "orbbec_lidar/types.hpp"
#include "types.hpp"

namespace ob_lidar::detail {
using OnDataCallback = std::function<void(const uint8_t* data, size_t size)>;
using namespace foonathan::memory::literals;

struct MemoryTracker {
    // NOTE: Do not change the name of the function
    void on_node_allocation(void* mem, std::size_t size, std::size_t) noexcept {
        LOG_INFO("{} node allocated: {} {} ", spdlog::fmt_lib::ptr(this),
                 spdlog::fmt_lib::ptr(mem), size);
    }

    void on_array_allocation(void* mem, std::size_t count, std::size_t size,
                             std::size_t) noexcept {
        LOG_INFO("{} array allocated: {} {} * {}", spdlog::fmt_lib::ptr(this),
                 spdlog::fmt_lib::ptr(mem), count, size);
    }

    void on_node_deallocation(void* ptr, std::size_t, std::size_t) noexcept {
        LOG_INFO("{} node deallocated: {}", spdlog::fmt_lib::ptr(this),
                 spdlog::fmt_lib::ptr(ptr));
    }

    void on_array_deallocation(void* ptr, std::size_t, std::size_t,
                               std::size_t) noexcept {
        LOG_INFO("{} array deallocated: {}", spdlog::fmt_lib::ptr(this),
                 spdlog::fmt_lib::ptr(ptr));
    }
};
// Channel abstraction
class Channel {
   public:
    Channel(size_t max_receive_buffer_size = 16_MiB);
    virtual ~Channel() = default;
    virtual void send(const std::vector<uint8_t>& buffer) = 0;
    virtual void setOnDataCallback(const OnDataCallback& callback) = 0;

   protected:
    using tracked_allocator_t =
        foonathan::memory::tracked_allocator<MemoryTracker,
                                             foonathan::memory::memory_pool<>>;
    tracked_allocator_t receive_buffer_pool_;
    foonathan::memory::vector<uint8_t, tracked_allocator_t> receive_buffer_;
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
}  // namespace ob_lidar::detail
