#include <filesystem>
#include <iostream>
#include <toml++/toml.hpp>
#include <uvw.hpp>

#include "orbbec_lidar/orbbec_lidar.hpp"
#include "tcp_server.hpp"
#include "udp_server.hpp"
namespace ob = ob_lidar;
int main() {
    std::shared_ptr<uvw::loop> loop = uvw::loop::get_default();

    auto tcp_server1 = std::make_shared<ob_test::TcpServer>(loop, "0.0.0.0", 2401);
    auto tcp_server2 = std::make_shared<ob_test::TcpServer>(loop, "0.0.0.0", 2402);

    loop->run();
    return 0;
}
