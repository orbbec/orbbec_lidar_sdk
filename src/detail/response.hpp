#pragma once

#include <spdlog/fmt/bundled/chrono.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <variant>
#include <vector>

#include "types.hpp"
#include "utils.hpp"

namespace ob_lidar {
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
    uint32_t packet_header1;  // 4D 53 02 F4
    uint16_t packet_header2;  // EB 90
    uint16_t frame_length;
    uint16_t start_angle;       // unit 0.01 degree
    uint16_t end_angle;         // unit 0.01 degree
    uint16_t angle_resolution;  // unit 0.001 degree

    uint8_t data_type;
    // block index 0-255
    uint8_t block_index;
    // frame index 0-65535
    uint16_t frame_index;
    // timestamp in us
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
    // spin speed, unit RPM
    uint16_t spin_speed;
    uint8_t reserved[5];
};
#pragma pack(pop)
using DataPacketHeader =
    std::variant<TL2401DataPacketHeader, MS600DataPacketHeader>;

class CommandResponse {
   public:
    CommandResponse() = default;

    Status setData(const uint8_t *data, size_t size) {
        if (size < sizeof(CommandResponseHeader)) {
            return Status::INVALID;
        }
        memcpy(&header_, data, sizeof(CommandResponseHeader));
        header_.header = ntohs(header_.header);
        header_.frame_length = ntohs(header_.frame_length);
        header_.command_id = ntohs(header_.command_id);
        header_.status = ntohs(header_.status);
        // read data
        data_.resize(header_.frame_length);
        memcpy(data_.data(), data + sizeof(CommandResponseHeader),
               data_.size());
        // read crc
        calc_crc_ = calcCrc8(data, size - 1);
        crc_ = data[size - 1];
        if (calc_crc_ != crc_) {
            std::cerr << "Invalid crc "
                      << "expect: " << static_cast<int>(calc_crc_)
                      << " actual: " << static_cast<int>(crc_) << std::endl;
            return Status::INVALID;
        }
        return Status::OK;
    }

    bool isValid() const { return calc_crc_ == crc_; }

    uint16_t getHeader() { return header_.header; }

    uint16_t getCommandId() { return header_.command_id; }

    [[nodiscard]] const std::vector<uint8_t> &getData() const { return data_; }

    size_t getDataSize() const { return data_.size(); }

    void getData(void *data, size_t size, size_t *read_size) {
        const size_t min_size = std::min(size, data_.size());
        *read_size = min_size;
        memcpy(data, data_.data(), min_size);
    }

    [[nodiscard]] uint8_t getStatus() const {
        // return low 8 bit
        return header_.status & 0xFF;
    }

   public:
    // parse from data
    CommandResponseHeader header_;
    std::vector<uint8_t> data_;
    uint8_t crc_;
    uint8_t calc_crc_;
};

static constexpr int LASER_SCAN_BLOCK_LENGTH_10HZ = 378;
static constexpr int LASER_SCAN_BLOCK_LENGTH_15HZ = 200;
// 240 * 96 = 23040
static constexpr int LASER_SCAN_BLOCK_LENGTH_20HZ_CALIB = 240;
static constexpr int LASER_SCAN_BLOCK_LENGTH_20HZ = 150;
static constexpr int LASER_SCAN_BLOCK_LENGTH_25HZ = 120;
static constexpr int LASER_SCAN_BLOCK_LENGTH_30HZ = 100;

static constexpr int LASER_SCAN_BLOCK_TYPE_10HZ = 0x00;
static constexpr int LASER_SCAN_BLOCK_TYPE_15HZ = 0x01;
static constexpr int LASER_SCAN_BLOCK_TYPE_20HZ = 0x02;
static constexpr int LASER_SCAN_BLOCK_TYPE_25HZ = 0x03;
static constexpr int LASER_SCAN_BLOCK_TYPE_30HZ = 0x04;

class DataResponse {
   public:
    explicit DataResponse(const LidarModel model) : model_(model) {}

    void MS600HeaderToHeader() {
        auto &header = std::get<MS600DataPacketHeader>(header_);
        header.packet_header1 = ntohl(header.packet_header1);
        header.packet_header2 = ntohs(header.packet_header2);
        header.frame_length = ntohs(header.frame_length);
        header.start_angle = ntohs(header.start_angle);
        header.end_angle = ntohs(header.end_angle);
        header.angle_resolution = ntohs(header.angle_resolution);
        header.frame_index = ntohs(header.frame_index);
        header.timestamp = ntohl(header.timestamp);
        header.sync_mode = header.sync_mode;
        header.special_mode = header.special_mode;
        header.warning_info = ntohl(header.warning_info);
        header.contaminated_angle = ntohs(header.contaminated_angle);
        header.contaminated_level = header.contaminated_level;
        header.apd_temperature = ntohs(header.apd_temperature);
        header.spin_speed = ntohs(header.spin_speed);
    }

    void TL2401HeaderToHeader() {
        auto &header = std::get<TL2401DataPacketHeader>(header_);
        header.packet_header1 = ntohl(header.packet_header1);
        header.packet_header2 = ntohs(header.packet_header2);
        header.data_length = ntohs(header.data_length);
        header.frame_index = ntohs(header.frame_index);
        header.timestamp = ntohl(header.timestamp);
        header.sync_mode = header.sync_mode;
        header.warning_status = ntohl(header.warning_status);
        header.echo_mode = header.echo_mode;
        header.horizontal_rpm = ntohs(header.horizontal_rpm);
        header.vertical_frequency = ntohs(header.vertical_frequency);
        header.apd_temperature = ntohs(header.apd_temperature);
    }

    Status setData(const uint8_t *data, size_t size) {
        if (model_ == LidarModel::MS600) {
            if (size < sizeof(MS600DataPacketHeader)) {
                return Status::INVALID;
            }
            MS600DataPacketHeader ms600_header;
            memcpy(&ms600_header, data, sizeof(MS600DataPacketHeader));
            header_ = ms600_header;
            // Important: convert to host order
            MS600HeaderToHeader();
            // cast timestamp us to ns
            auto &header = std::get<MS600DataPacketHeader>(header_);
            timestamp_ = static_cast<uint64_t>(header.timestamp) * 1000;
            auto data_type = header.data_type;
            size_t point_size = 0;
            if (data_type == LASER_SCAN_BLOCK_TYPE_10HZ) {
                point_size = LASER_SCAN_BLOCK_LENGTH_10HZ;
            } else if (data_type == LASER_SCAN_BLOCK_TYPE_15HZ) {
                point_size = LASER_SCAN_BLOCK_LENGTH_15HZ;
            } else if (data_type == LASER_SCAN_BLOCK_TYPE_20HZ) {
                point_size = LASER_SCAN_BLOCK_LENGTH_20HZ;
            } else if (data_type == LASER_SCAN_BLOCK_TYPE_25HZ) {
                point_size = LASER_SCAN_BLOCK_LENGTH_25HZ;
            } else if (data_type == LASER_SCAN_BLOCK_TYPE_30HZ) {
                point_size = LASER_SCAN_BLOCK_LENGTH_30HZ;
            } else {
                return Status::INVALID;
            }
            data_.resize(point_size * 4);
            memcpy(data_.data(), data + sizeof(MS600DataPacketHeader),
                   data_.size());
            memcpy(&end, data + sizeof(MS600DataPacketHeader) + data_.size(),
                   sizeof(uint32_t));
        } else if (model_ == LidarModel::TL2401) {
            if (size < sizeof(TL2401DataPacketHeader)) {
                return Status::INVALID;
            }
            TL2401DataPacketHeader tl2401_header;
            memcpy(&tl2401_header, data, sizeof(TL2401DataPacketHeader));
            header_ = tl2401_header;
            // Important: convert to host order
            TL2401HeaderToHeader();

            timestamp_ = std::get<TL2401DataPacketHeader>(header_).timestamp;
            size_t data_length =
                std::get<TL2401DataPacketHeader>(header_).data_length;
            data_.resize(data_length);
            memcpy(data_.data(), data + sizeof(TL2401DataPacketHeader),
                   data_.size());
            memcpy(&end, data + sizeof(TL2401DataPacketHeader) + data_.size(),
                   sizeof(uint32_t));
        }
        return Status::OK;
    }

    void getData(void *data, const size_t size) const {
        if (size > data_.size()) {
            throw std::invalid_argument("size is larger than data size");
        }
        memcpy(data, data_.data(), size);
    }

    [[nodiscard]] const std::vector<uint8_t> &getData() const { return data_; }

    [[nodiscard]] const DataPacketHeader &getHeader() const { return header_; }

    [[nodiscard]] uint8_t getDataType() const {
        return std::visit([](auto &&h) { return h.data_type; }, header_);
    }
    [[nodiscard]] uint16_t getFrameIndex() const {
        return std::visit([](auto &&h) { return h.frame_index; }, header_);
    }
    [[nodiscard]] uint64_t getTimestamp() const { return timestamp_; }

   private:
    LidarModel model_;
    uint64_t timestamp_ = 0;
    DataPacketHeader header_;
    std::vector<uint8_t> data_;
    uint32_t end = 0;
};

}  // namespace ob_lidar
