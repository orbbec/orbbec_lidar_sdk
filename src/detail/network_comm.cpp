#include "network_comm.hpp"

#include <check.hpp>

#include "../logger.hpp"

namespace ob_lidar::detail {

using pool_allocator_t =
    foonathan::memory::memory_pool<foonathan::memory::node_pool,
                                   foonathan::memory::heap_allocator>;

Channel::Channel(size_t max_receive_buffer_size)
    : receive_buffer_pool_(foonathan::memory::make_tracked_allocator(
          MemoryTracker{},
          // static storage of size max_receive_buffer_size
          pool_allocator_t(foonathan::memory::list_node_size<uint8_t>::value,
                           max_receive_buffer_size))),
      receive_buffer_(receive_buffer_pool_) {
    LOG_INFO("Channel created");
}

TCPChannel::TCPChannel(std::shared_ptr<uvw::loop> loop, const std::string& ip,
                       uint32_t port) {
    handle_ = loop->resource<uvw::tcp_handle>();
    socket_address_ = uvw::socket_address{ip, port};
    handle_->on<uvw::error_event>(
        [](const auto&, auto& udp_handle) { udp_handle.close(); });
    handle_->on<uvw::connect_event>(
        [](const uvw::connect_event&, uvw::tcp_handle& handle) {
            handle.read();
        });
    handle_->connect(socket_address_);
}

TCPChannel::~TCPChannel() = default;

void TCPChannel::send(const std::vector<uint8_t>& buffer) {
    const auto data = reinterpret_cast<const char*>(buffer.data());
    CHECK_NOTNULL(handle_);
    handle_->write(const_cast<char*>(data), buffer.size());
}

void TCPChannel::setOnDataCallback(const OnDataCallback& callback) {
    callback_ = callback;
    handle_->on<uvw::data_event>(
        [this](const uvw::data_event& event, uvw::tcp_handle&) {
            if (callback_) {
                // TODO: use memory pool to avoid memory allocation
                receive_buffer_.assign(event.data.get(),
                                       event.data.get() + event.length);
                callback_(receive_buffer_.data(), receive_buffer_.size());
            }
        });

    handle_->on<uvw::end_event>(
        [](const uvw::end_event&, uvw::tcp_handle& handle) { handle.close(); });

    handle_->on<uvw::close_event>(
        [](const uvw::close_event&, uvw::tcp_handle& handle) {
            handle.close();
        });
}

UDPChannel::UDPChannel(std::shared_ptr<uvw::loop> loop, const std::string& ip,
                       uint32_t port) {
    handle_ = loop->resource<uvw::udp_handle>();
    socket_address_ = uvw::socket_address{ip, port};
    handle_->bind("0.0.0.0", 0);
    handle_->recv();
}

UDPChannel::~UDPChannel() = default;

void UDPChannel::send(const std::vector<uint8_t>& buffer) {
    const auto data = reinterpret_cast<const char*>(buffer.data());
    handle_->send(socket_address_, const_cast<char*>(data), buffer.size());
}

void UDPChannel::setOnDataCallback(const OnDataCallback& callback) {
    callback_ = callback;

    handle_->on<uvw::error_event>([](const auto&, auto&) {});

    handle_->on<uvw::udp_data_event>(
        [this](const uvw::udp_data_event& event, uvw::udp_handle&) {
            if (callback_) {
                // TODO: use memory pool to avoid memory allocation
                receive_buffer_.assign(event.data.get(),
                                       event.data.get() + event.length);
                callback_(receive_buffer_.data(), receive_buffer_.size());
            }
        });
}

// NetworkComm implementation
NetworkComm::NetworkComm(LidarProtocolType protocol)
    : protocol_(protocol), loop_(uvw::loop::create()) {}

NetworkComm::~NetworkComm() {
    if (loop_thread_ && loop_thread_->joinable()) {
        loop_->stop();
        loop_->walk([](auto&& handle) { handle.close(); });
        loop_->close();
        // FIXME: Join will be stuck, Do know why
        // TODO: Need to investigate
        loop_thread_->detach();
    }
}

void NetworkComm::createChannel(LidarChannelType type, const std::string& ip,
                                uint32_t port) {
    LOG_INFO("Create channel: type: {}, ip: {}, port: {}",
             magic_enum::enum_name(type), ip, port);
    if (protocol_ == LidarProtocolType::TCP) {
        channels_[type] = std::make_unique<TCPChannel>(loop_, ip, port);
    } else {
        channels_[type] = std::make_unique<UDPChannel>(loop_, ip, port);
    }
}

void NetworkComm::removeChannel(LidarChannelType type) {
    if (channels_.count(type)) {
        channels_.erase(type);
    }
}

void NetworkComm::sendData(LidarChannelType type,
                           const std::vector<uint8_t>& buffer) {
    if (channels_.count(type)) {
        channels_[type]->send(buffer);
    }
}

void NetworkComm::setOnDataCallback(LidarChannelType type,
                                    const OnDataCallback& callback) {
    if (channels_.count(type)) {
        channels_[type]->setOnDataCallback(callback);
    }
}

void NetworkComm::createSingleChannel(const std::string& ip, uint32_t port) {
    is_single_channel_mode_ = true;
    LOG_INFO("Create single channel: protocol: {}, ip: {}, port: {}",
             magic_enum::enum_name(protocol_), ip, port);
    if (protocol_ == LidarProtocolType::TCP) {
        single_channel_ = std::make_unique<TCPChannel>(loop_, ip, port);
    } else {
        single_channel_ = std::make_unique<UDPChannel>(loop_, ip, port);
    }
}

void NetworkComm::sendData(const std::vector<uint8_t>& buffer) const {
    if (is_single_channel_mode_ && single_channel_) {
        single_channel_->send(buffer);
    }
}

void NetworkComm::runLoop() {
    loop_thread_ = std::make_shared<std::thread>([this]() { loop_->run(); });
}

void NetworkComm::setOnDataCallback(const OnDataCallback& callback) const {
    if (is_single_channel_mode_ && single_channel_) {
        single_channel_->setOnDataCallback(callback);
    }
}

// NetworkCommFactory implementation
std::unique_ptr<NetworkComm> NetworkCommFactory::create(
    const std::shared_ptr<NetworkConfig>& config) {
    LidarProtocolType protocol_type = config->getProtocolType();
    assert(protocol_type != LidarProtocolType::UNKNOWN);
    auto network_comm = std::make_unique<NetworkComm>(protocol_type);

    const std::string ip = config->getIp();

    if (config->isSinglePort()) {
        network_comm->createSingleChannel(ip, config->getPort());
    } else {
        for (const auto& channel :
             magic_enum::enum_values<LidarChannelType>()) {
            if (config->hasChannel(channel)) {
                auto port = config->getPort(channel);
                network_comm->createChannel(channel, ip, port);
            }
        }
    }
    network_comm->runLoop();
    return network_comm;
}

}  // namespace ob_lidar::detail
