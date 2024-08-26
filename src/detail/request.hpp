#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>
#if defined(__unix__)
#include <arpa/inet.h>
#elif defined(_WIN32)
#include <winsock2.h>
#endif
#include "types.hpp"
#include "utils.hpp"
namespace ob_lidar {
class NetworkPacket {
   public:
    NetworkPacket(uint16_t header, uint8_t protocol_version,
                  uint16_t command_id, uint16_t command_type)
        : header_(htons(header)),
          protocol_version_(protocol_version),
          command_id_(htons(command_id)),
          command_type_(htons(command_type)) {}

    void setPayload(const void* data, uint16_t payload_size) {
        payload_.resize(payload_size);
        std::memcpy(payload_.data(), data, payload_size);
    }

    std::vector<uint8_t> build() const {
        std::vector<uint8_t> packet;
        uint16_t total_length = sizeof(header_) + sizeof(protocol_version_) +
                                sizeof(command_id_) + sizeof(command_type_) +
                                payload_.size() +
                                sizeof(uint8_t);  // +1 for CRC
        packet.reserve(total_length);

        // insert header
        packet.insert(
            packet.end(), reinterpret_cast<const uint8_t*>(&header_),
            reinterpret_cast<const uint8_t*>(&header_) + sizeof(header_));

        // insert protocol version
        packet.push_back(protocol_version_);
        uint16_t payload_length = htons(payload_.size());

        // insert payload length
        packet.insert(packet.end(),
                      reinterpret_cast<const uint8_t*>(&payload_length),
                      reinterpret_cast<const uint8_t*>(&payload_length) +
                          sizeof(payload_length));

        // insert command id
        packet.insert(packet.end(),
                      reinterpret_cast<const uint8_t*>(&command_id_),
                      reinterpret_cast<const uint8_t*>(&command_id_) +
                          sizeof(command_id_));

        // insert command type
        packet.insert(packet.end(),
                      reinterpret_cast<const uint8_t*>(&command_type_),
                      reinterpret_cast<const uint8_t*>(&command_type_) +
                          sizeof(command_type_));

        // insert data
        packet.insert(packet.end(), payload_.begin(), payload_.end());

        // calculate CRC
        uint8_t crc = calcCrc8(packet.data(), packet.size());
        packet.push_back(crc);

        return packet;
    }

   private:
    uint16_t header_;
    uint8_t protocol_version_;
    uint16_t command_id_;
    uint16_t command_type_;
    std::vector<uint8_t> payload_;
};

struct Request {
    explicit Request(uint16_t command_id) : command_id_(command_id) {}

    std::vector<uint8_t> serialize(const void* data, size_t payload_size) {
        NetworkPacket packet(header_, protocol_version_, command_id_, 0);
        packet.setPayload(data, payload_size);
        return packet.build();
    }

    void setHeader(uint16_t header) { header_ = header; }

    void setProtocolVersion(uint8_t protocol_version) {
        protocol_version_ = protocol_version;
    }

   private:
    uint16_t command_id_;
    uint16_t header_ = 0x01FE;
    uint8_t protocol_version_ = 0x01;
};

}  // namespace ob_lidar
