#pragma once
#include <string>

#include "types.hpp"

namespace ob_lidar {
std::string commandIDToString(uint16_t command_id);

uint8_t calcCrc8(const uint8_t *p, uint8_t len);

}  // namespace ob_lidar
