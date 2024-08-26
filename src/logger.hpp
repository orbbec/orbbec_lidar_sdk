#pragma once

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <iostream>

#include "orb_lidar_driver/config.hpp"

// Debug trace information, only valid when compiled into debug version
#define LOG_TRACE(...) \
    SPDLOG_LOGGER_TRACE(spdlog::default_logger(), __VA_ARGS__)

// Debugging information, for SDK developers, provides SDK internal running
// process and status information
#define LOG_DEBUG(...) \
    SPDLOG_LOGGER_DEBUG(spdlog::default_logger(), __VA_ARGS__)

// General information, facing users, providing SDK running status and call
// result information
#define LOG_INFO(...) SPDLOG_LOGGER_INFO(spdlog::default_logger(), __VA_ARGS__)

// Warning information (such as: memory usage reaches the maximum limit, cache
// queue is full, system memory is about to be exhausted, etc.)
#define LOG_WARN(...) SPDLOG_LOGGER_WARN(spdlog::default_logger(), __VA_ARGS__)

// Error information (such as: user parameter error, device error calling
// sequence, device offline and unresponsive, etc.)
#define LOG_ERROR(...) \
    SPDLOG_LOGGER_ERROR(spdlog::default_logger(), __VA_ARGS__)

// Fatal error message (for example: insufficient system memory prevents normal
// operation)
#define LOG_FATAL(...) \
    SPDLOG_LOGGER_CRITICAL(spdlog::default_logger(), __VA_ARGS__)

namespace ob_lidar_driver {

class Logger {
   public:
    Logger() = delete;
    ~Logger();

    explicit Logger(std::shared_ptr<LoggerConfig> config);

   private:
    // MUST be static, otherwise the logger will crash
    const static spdlog::details::registry &registry_;
    std::shared_ptr<Logger> instance_;
    std::once_flag init_flag_;
    std::mutex mutex_;
};

}  // namespace ob_lidar_driver
