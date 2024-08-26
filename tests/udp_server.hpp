#pragma once
#include <atomic>
#include <string>
#include <thread>
#include <uvw.hpp>
#include <vector>

namespace ob_test {
class UdpServer {
   public:
    explicit UdpServer(std::shared_ptr<uvw::loop> loop, const std::string& ip,
                       uint16_t port);

    ~UdpServer();

    void onDataReceived(const std::vector<uint8_t>& data,
                        uvw::udp_handle& handle);

    void sendFakeData();

    std::vector<uint8_t> buildResponseData(const uint16_t& command_id);

  void onResponseData(const uint16_t& command_id, uvw::udp_handle& handle);

   private:
    std::shared_ptr<uvw::loop> loop_ = nullptr;
    std::shared_ptr<uvw::udp_handle> server_ = nullptr;
    std::shared_ptr<uvw::udp_handle> client_handle_ = nullptr;
    std::string ip_;
    uint16_t port_ = 0;
    std::string peer_ip_;
    uint16_t peer_port_ = 0;
    // push data to client
    std::atomic_bool send_data_{false};
    std::shared_ptr<std::thread> send_thread_ = nullptr;
    std::vector<uint8_t> send_data_buffer_;
  // mock lidar params
  std::string fake_ip_ = "192.168.1.100";
  std::string fake_subnet_mask_ = "255.255.255.0";
  uint16_t fake_port_ = 2401;
  std::string fake_mac_addr_ = "00:11:22:33:44:55";
  // 16 bytes fake SN
  std::string fake_serial_number_ = "1234567890abcdfg";
};
}  // namespace ob_test
