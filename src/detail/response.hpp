#pragma once

#include <FastCRC.h>
#include <spdlog/fmt/bundled/chrono.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <variant>
#include <vector>

#include "types.hpp"
#include "utils.hpp"

namespace ob_lidar_driver {
#pragma pack(push, 1)

struct CommandResponseHeader {
    uint16_t header;           // 2 bit
    uint8_t protocol_version;  // 1 bit
    uint16_t frame_length;     // 2 bit
    uint16_t command_id;       // 2 bit
    uint16_t status;           // 2 bit
};

struct TL2401DataPacketHeader {
    // Packet header identifier (example: 4D 53 01 F4)
    uint32_t packet_header1;
    // Packet ID (example: EB 90)
    uint16_t packet_header2;
    // Total length of the data
    uint16_t data_length;

    uint8_t lidar_type;
    // Scan frequency type
    // 01: 5.5Hz scan frequency point cloud frame
    // 02: 11Hz scan frequency point cloud frame
    // 03: 16.5Hz scan frequency point cloud frame
    // 04: 22Hz scan frequency point cloud frame
    uint8_t scan_frequency_type;
    // Number of blocks in the packet
    uint8_t block_count;
    // Frame index number for synchronization
    uint16_t frame_index;
    uint8_t block_data_num;
    uint8_t data_type;  // 0: IMU, 1. PointCloud, 2. Sphere Point Cloud
    // Timestamp of the data in nanoseconds
    uint64_t timestamp;
    // Synchronization mode identifier
    uint8_t sync_mode;  // 0 free run , 1, external sync, 2, PTP sync
    uint32_t warning_status;
    uint8_t echo_mode;
    uint16_t horizontal_rpm;
    uint16_t vertical_frequency;  // unit 0.1Hz
    uint16_t apd_temperature;     // unit 0.01C
    // Reserved for future use or padding
    uint8_t reserved[5];
};

struct MS600DataPacketHeader {
    uint32_t frame_head1;  // 4D 53 01 F4
    uint16_t frame_head2;  // EB 90
    uint16_t frame_length;
    uint16_t start_angle;       // unit 0.01 degree
    uint16_t end_angle;         // unit 0.01 degree
    uint16_t angle_resolution;  // unit 0.001 degree

    uint8_t data_type;
    // block index 0-18
    uint8_t block_index;
    // frame index 0-65535
    uint16_t frame_index;
    uint32_t timestamp;
    // sync mode 0: free run, 1: external sync.
    uint8_t sync_mode;
    // special mode 0: normal, 1: contamination detection
    uint8_t special_mode;
    // warning info
    uint32_t warning_info;
    // contaminated angle
    uint16_t contaminated_angle;
    // contaminated level
    uint8_t contaminated_level;
    // apd temperature, unit 1 degree
    uint16_t apd_temperature;
    // motor speed, unit RPM
    uint16_t motor_speed;
    uint8_t reserved[5];
};
#pragma pack(pop)
using DataPacketHeader =
    std::variant<TL2401DataPacketHeader, MS600DataPacketHeader>;

class CommandResponse {
   public:
    CommandResponse() = default;

    Status setData(std::vector<uint8_t> data) {
        if (data.size() < sizeof(CommandResponseHeader)) {
            return Status::INVALID;
        }
        memcpy(&header_, data.data(), sizeof(CommandResponseHeader));
        header_.header = ntohs(header_.header);
        header_.frame_length = ntohs(header_.frame_length);
        header_.command_id = ntohs(header_.command_id);
        header_.status = ntohs(header_.status);
        // read data
        data_.resize(header_.frame_length);
        memcpy(data_.data(), data.data() + sizeof(CommandResponseHeader),
               data_.size());
        // read crc
        crc_ = data[data.size() - 1];
        if (!isValid()) {
            std::cerr << "Invalid crc" << std::endl;
            return Status::INVALID;
        }
        return Status::OK;
    }

    bool isValid() {
        // check crc
        uint8_t calc_crc = crc8_.smbus(data_.data(), data_.size());
        return crc_ == calc_crc;
    }

    uint16_t getHeader() { return header_.header; }

    uint16_t getCommandId() { return header_.command_id; }

    [[nodiscard]] const std::vector<uint8_t> &getData() const { return data_; }

    size_t getDataSize() const { return data_.size(); }

    void getData(void *data, size_t size, size_t *read_size) {
        const size_t min_size = std::min(size, data_.size());
        *read_size = min_size;
        memcpy(data, data_.data(), min_size);
    }

    [[nodiscard]] uint16_t getStatus() const { return header_.status; }

   public:
    // parse from data
    FastCRC8 crc8_;
    CommandResponseHeader header_;
    std::vector<uint8_t> data_;
    uint8_t crc_;
};

class DataResponse {
   public:
    explicit DataResponse(const LidarModel model) : model_(model) {}

    Status setData(const std::vector<uint8_t> &data) {
        if (data.size() < sizeof(DataPacketHeader)) {
            return Status::INVALID;
        }
        memcpy(&header_, data.data(), sizeof(DataPacketHeader));
        size_t data_length = 0;
        if (model_ == LidarModel::MS600) {
            // FIXME: map frame_length to data_length
            data_length = std::get<MS600DataPacketHeader>(header_).frame_length;
        } else if (model_ == LidarModel::TL2401) {
            data_length = std::get<TL2401DataPacketHeader>(header_).data_length;
        }
        data_.resize(data_length);
        memcpy(data_.data(), data.data() + sizeof(DataPacketHeader),
               data_.size());
        memcpy(&end, data.data() + sizeof(DataPacketHeader) + data_.size(),
               sizeof(uint32_t));
        return Status::OK;
    }

    void getData(void *data, const size_t size) const {
        if (size > data_.size()) {
            throw std::invalid_argument("size is larger than data size");
        }
        memcpy(data, data_.data(), size);
    }

    const std::vector<uint8_t> &getData() const { return data_; }

    uint8_t getDataType() const {
        return std::visit([](auto &&h) { return h.data_type; }, header_);
    }
    uint16_t getFrameIndex() const {
        return std::visit([](auto &&h) { return h.frame_index; }, header_);
    }
    uint64_t getTimestamp() const {
        return std::visit([](auto &&h) -> uint64_t {
            if constexpr (std::is_same_v<std::decay_t<decltype(h.timestamp)>, uint32_t>) {
                return static_cast<uint64_t>(h.timestamp);
            } else {
                return h.timestamp;it
            }
        }, header_);
    }


   private:
    LidarModel model_;
    DataPacketHeader header_;
    std::vector<uint8_t> data_;
    uint32_t end = 0;
};

}  // namespace ob_lidar_driver
