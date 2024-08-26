#include "udp_server.hpp"

#include <FastCRC.h>

#include <cstring>
#include <iostream>
#include <magic_enum/magic_enum.hpp>
#include <memory>
#include <orb_lidar_driver/option.hpp>
#include <string>

#include "detail/request.hpp"
#include "detail/response.hpp"
#include "detail/utils.hpp"
#include "test_utils.hpp"

namespace ob = ob_lidar_driver;
namespace ob_test {
UdpServer::UdpServer(std::shared_ptr<uvw::loop> loop, const std::string &ip,
                     uint16_t port)
    : loop_(std::move(loop)), ip_(ip), port_(port) {
    server_ = loop_->resource<uvw::udp_handle>();

    server_->on<uvw::error_event>(
        [](const uvw::error_event &, uvw::udp_handle &) {
            std::cerr << "Server Error!" << std::endl;
        });
    server_->on<uvw::udp_data_event>(
        [this](const uvw::udp_data_event &event, uvw::udp_handle &handle) {
            std::vector<uint8_t> received_data(event.data.get(),
                                               event.data.get() + event.length);
            peer_ip_ = event.sender.ip;
            peer_port_ = event.sender.port;
            std::cout << "Received data from " << peer_ip_ << ":" << peer_port_
                      << std::endl;
            onDataReceived(received_data, handle);
        });
    if (const auto res = server_->bind(ip, port); res < 0) {
        throw std::runtime_error("Failed to bind to " + ip + ":" +
                                 std::to_string(port));
    }
    server_->recv();
    client_handle_ = loop_->resource<uvw::udp_handle>();
}

UdpServer::~UdpServer() {
    send_data_ = false;
    if (send_thread_ && send_thread_->joinable()) {
        send_thread_->join();
    }
}

#define COMMAND_TYPE_RESPONSE_HEADER 0xFE01
#define DATA_TYPE_RESPONSE_HEADER 0x534D

#pragma pack(push, 1)
struct DataHeader {
    uint16_t header;
    uint8_t protocol_version;
    uint16_t payload_length;
    uint16_t command_id;
    uint16_t command_type;
};
#pragma pack(pop)

std::vector<uint8_t> UdpServer::buildResponseData(const uint16_t &command_id) {
    std::vector<uint8_t> payload;
    using namespace ob_lidar_driver;
    int int_res = 0;

    std::string string_res;

    if (const uint16_t high_byte = (command_id >> 8) & 0xFF;
        high_byte == 0x01 || high_byte == 0x03) {
        // set command
        return payload;
    } else if (high_byte == 0x02 || high_byte == 0x04) {
        // set command;
        switch (command_id) {
            case GET_IP_ADDR:
                int_res = ipAddressToInt(fake_ip_);
                payload.insert(
                    payload.end(), reinterpret_cast<uint8_t *>(&int_res),
                    reinterpret_cast<uint8_t *>(&int_res) + sizeof(int_res));
                return payload;
            case GET_PORT:
                int_res = fake_port_;
                payload.insert(
                    payload.end(), reinterpret_cast<uint8_t *>(&int_res),
                    reinterpret_cast<uint8_t *>(&int_res) + sizeof(int_res));
                return payload;
            case GET_MAC_ADDR:
                string_res = fake_mac_addr_;
                payload.insert(payload.end(), string_res.begin(),
                               string_res.end());
                return payload;
            case GET_SUBNET_MASK:
                int_res = ipAddressToInt(fake_subnet_mask_);
                payload.insert(
                    payload.end(), reinterpret_cast<uint8_t *>(&int_res),
                    reinterpret_cast<uint8_t *>(&int_res) + sizeof(int_res));
                return payload;
            case GET_FPGA_VERSION:
                string_res = "1.0.0";
                payload.insert(payload.end(), string_res.begin(),
                               string_res.end());
                return payload;
            case GET_SERIAL_NUMBER:
                string_res = fake_serial_number_;
                payload.insert(payload.end(), string_res.begin(),
                               string_res.end());
                return payload;
            case GET_FIRMWARE_VERSION:
                string_res = "1.0.0";
                payload.insert(payload.end(), string_res.begin(),
                               string_res.end());
                return payload;
            case GET_ECHO_MODE:
                int_res = 0;
                payload.insert(
                    payload.end(), reinterpret_cast<uint8_t *>(&int_res),
                    reinterpret_cast<uint8_t *>(&int_res) + sizeof(int_res));
                return payload;
            case GET_PRODUCT_MODEL:
                int_res = 0;
                payload.insert(
                    payload.end(), reinterpret_cast<uint8_t *>(&int_res),
                    reinterpret_cast<uint8_t *>(&int_res) + sizeof(int_res));
            case GET_WORK_MODE:
                int_res = 2;
                payload.insert(
                    payload.end(), reinterpret_cast<uint8_t *>(&int_res),
                    reinterpret_cast<uint8_t *>(&int_res) + sizeof(int_res));
                return payload;
            case GET_SPIN_SPEED:
                int_res = 1000;
                payload.insert(
                    payload.end(), reinterpret_cast<uint8_t *>(&int_res),
                    reinterpret_cast<uint8_t *>(&int_res) + sizeof(int_res));
                return payload;
            case GET_TX_VOLTAGE:
                int_res = 3400;
                payload.insert(
                    payload.end(), reinterpret_cast<uint8_t *>(&int_res),
                    reinterpret_cast<uint8_t *>(&int_res) + sizeof(int_res));
                return payload;
            default:
                std::cerr << "unhandled command "
                          << commandIDToString(command_id) << std::endl;
                return payload;
        }
    } else {
        std::cerr << "Invalid command id " << command_id << std::endl;
        return payload;
    }
}

void UdpServer::onDataReceived(const std::vector<uint8_t> &data,
                               uvw::udp_handle &handle) {
    if (data.size() < sizeof(DataHeader)) {
        std::cerr << "Invalid data received" << std::endl;
        return;
    }
    DataHeader header{};
    std::memcpy(&header, data.data(), sizeof(DataHeader));
    if (ntohs(header.header) != COMMAND_TYPE_RESPONSE_HEADER) {
        std::cerr << "Invalid data header" << std::endl;
        return;
    }
    auto payload_length = ntohs(header.payload_length);
    if (data.size() < sizeof(DataHeader) + payload_length) {
        std::cerr << "Invalid data length" << std::endl;
        return;
    }
    std::vector<uint8_t> payload(
        data.begin() + sizeof(DataHeader),
        data.begin() + sizeof(DataHeader) + payload_length);

    const uint8_t crc = data.back();
    FastCRC8 CRC8_;
    if (crc != CRC8_.smbus(data.data(), data.size() - 1)) {
        std::cerr << "Invalid CRC" << std::endl;
        return;
    }
    uint16_t command_id = ntohs(header.command_id);
    auto option = static_cast<ob::LidarOption>(command_id);
    std::cout << "Received command: " << ob::commandIDToString(command_id)
              << std::endl;
    if (option == ob::OB_LIDAR_OPTION_ENABLE_STREAMING) {
        uint32_t enable = 0;
        std::memcpy(&enable, payload.data(), sizeof(uint32_t));
        if (enable && send_thread_ == nullptr) {
            send_data_ = true;
            send_thread_ =
                std::make_shared<std::thread>(&UdpServer::sendFakeData, this);
        } else if (!enable && send_thread_ != nullptr) {
            send_data_ = false;
            send_thread_->join();
            send_thread_ = nullptr;
        }
    }
    onResponseData(command_id, handle);
}

void UdpServer::onResponseData(const uint16_t &command_id,
                               uvw::udp_handle &handle) {
    std::vector<uint8_t> response_data;
    uint16_t total_length =
        sizeof(DataHeader) + sizeof(uint8_t) + sizeof(uint32_t);
    response_data.reserve(total_length);
    DataHeader response_header{};
    response_header.header = htons(COMMAND_TYPE_RESPONSE_HEADER);
    response_header.protocol_version = 0x01;
    auto payload_data = buildResponseData(command_id);
    const uint16_t payload_size = payload_data.size();
    response_header.payload_length = htons(payload_size);
    response_header.command_id = htons(command_id);
    response_header.command_type = htons(0);
    response_data.insert(response_data.end(),
                         reinterpret_cast<const uint8_t *>(&response_header),
                         reinterpret_cast<const uint8_t *>(&response_header) +
                             sizeof(DataHeader));
    response_data.insert(response_data.end(), payload_data.begin(),
                         payload_data.end());
    FastCRC8 CRC8;
    const uint8_t resp_crc =
        CRC8.smbus(payload_data.data(), payload_data.size());
    response_data.push_back(resp_crc);
    const auto response = reinterpret_cast<const char *>(response_data.data());
    handle.send(peer_ip_, peer_port_, const_cast<char *>(response),
                response_data.size());
}
void UdpServer::sendFakeData() {
    std::vector<ob::LidarPoint> fake_points;
    ob::LidarPoint fake_point{};
    fake_point.x = 1;
    fake_point.y = 2;
    fake_point.z = 3;
    fake_point.reflectivity = 4;
    fake_point.tag = 5;
    for (int i = 0; i < 10; i++) {
        fake_points.push_back(fake_point);
    }
    uint16_t frame_length = sizeof(ob::LidarPoint) * fake_points.size();
    const uint16_t total_length =
        sizeof(ob::DataPacketHeader) + frame_length + sizeof(uint8_t);
    send_data_buffer_.reserve(total_length);
    // insert header
    ob::DataPacketHeader header{};
    std::get<ob::TL2401DataPacketHeader>(header).frame_index = htons(0);
    std::get<ob::TL2401DataPacketHeader>(header).data_type = 0x02;
    std::get<ob::TL2401DataPacketHeader>(header).packet_header1 =
        htons(DATA_TYPE_RESPONSE_HEADER);
    std::get<ob::TL2401DataPacketHeader>(header).packet_header2 = htons(0xEB90);
    std::get<ob::TL2401DataPacketHeader>(header).data_length =
        htons(frame_length);
    send_data_buffer_.insert(send_data_buffer_.end(),
                             reinterpret_cast<const uint8_t *>(&header),
                             reinterpret_cast<const uint8_t *>(&header) +
                                 sizeof(ob::DataPacketHeader));
    for (const auto &point : fake_points) {
        send_data_buffer_.insert(
            send_data_buffer_.end(), reinterpret_cast<const uint8_t *>(&point),
            reinterpret_cast<const uint8_t *>(&point) + sizeof(ob::LidarPoint));
    }
    while (send_data_) {
        const auto data =
            reinterpret_cast<const char *>(send_data_buffer_.data());

        client_handle_->send(uvw::socket_address{peer_ip_, peer_port_},
                             const_cast<char *>(data),
                             send_data_buffer_.size());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
}  // namespace ob_test
