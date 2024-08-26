#include "logger.hpp"

#include <spdlog/async.h>
#include <spdlog/async_logger.h>
#include <spdlog/sinks/rotating_file_sink.h>

#include <iomanip>
namespace ob_lidar {

const spdlog::details::registry &Logger::registry_ =
    spdlog::details::registry::instance();
// default
const static LogLevel DEFAULT_CONSOLE_LOG_LEVEL = LogLevel::INFO;
const static LogLevel DEFAULT_FILE_LOG_LEVEL = LogLevel::INFO;
const static bool LOG_TO_CONSOLE = true;
const static bool LOG_TO_FILE = true;
const static size_t DEFAULT_MAX_FILE_SIZE_MB = 10;
const static size_t DEFAULT_MAX_FILE_NUM = 10;
const static std::string DEFAULT_LOG_FILE_DIR = "./logs";
const static std::string DEFAULT_LOG_FILE_PATTERN =
    "[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%g:%#] %v";

spdlog::level::level_enum LogLevelToSpdlogLevel(LogLevel log_level) {
    switch (log_level) {
        case LogLevel::TRACE:
            return spdlog::level::trace;
        case LogLevel::DEBUG:
            return spdlog::level::debug;
        case LogLevel::INFO:
            return spdlog::level::info;
        case LogLevel::WARNING:
            return spdlog::level::warn;
        case LogLevel::ERROR:
            return spdlog::level::err;
        case LogLevel::CRITICAL:
            return spdlog::level::critical;
        default:
            return spdlog::level::info;
    }
}

Logger::~Logger() {
    std::cout << "Logger destroyed" << std::endl;
    spdlog::shutdown();
}

Logger::Logger(const std::shared_ptr<LoggerConfig> config) {
    spdlog::level::level_enum console_log_level =
        LogLevelToSpdlogLevel(DEFAULT_CONSOLE_LOG_LEVEL);
    spdlog::level::level_enum file_log_level =
        LogLevelToSpdlogLevel(DEFAULT_FILE_LOG_LEVEL);
    bool log_to_console = LOG_TO_CONSOLE;
    bool log_to_file = LOG_TO_FILE;
    size_t max_file_size_mb = DEFAULT_MAX_FILE_SIZE_MB;
    size_t max_file_num = DEFAULT_MAX_FILE_NUM;
    std::string log_file_dir = DEFAULT_LOG_FILE_DIR;

    if (config != nullptr) {
        console_log_level = LogLevelToSpdlogLevel(config->getConsoleLogLevel());
        file_log_level = LogLevelToSpdlogLevel(config->getFileLogLevel());
        log_to_console = config->shouldLogToConsole();
        log_to_file = config->shouldLogToFile();
        max_file_size_mb = config->getMaxLogFileSize();
        max_file_num = config->getMaxLogFileNum();
        log_file_dir = config->getLogFileDir();
    }

    // create console sink
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(console_log_level);
    console_sink->set_pattern(DEFAULT_LOG_FILE_PATTERN);

    // create file sink
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm;
    localtime_r(&now_time_t, &now_tm);
    std::ostringstream oss;
    oss << std::put_time(&now_tm, "%Y-%m-%d_%H-%M-%S");
    std::string log_file_name = "ob_lidar_" + oss.str();
    std::string logger_name = "ob_lidar";

    auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        log_file_dir + "/" + log_file_name + ".log",
        max_file_size_mb * 1024 * 1024, max_file_num);
    file_sink->set_level(file_log_level);
    file_sink->set_pattern(DEFAULT_LOG_FILE_PATTERN);

    // create logger
    std::shared_ptr<spdlog::logger> logger;
    if (config && config->enableAsync()) {
        spdlog::init_thread_pool(1024, 4);
        logger = std::make_shared<spdlog::async_logger>(
            logger_name, spdlog::sinks_init_list{console_sink, file_sink},
            spdlog::thread_pool(), spdlog::async_overflow_policy::block);
        spdlog::flush_every(std::chrono::seconds(1));
    } else {
        logger = std::make_shared<spdlog::logger>(
            logger_name, spdlog::sinks_init_list{console_sink, file_sink});
    }
    logger->set_level(spdlog::level::trace);
    set_default_logger(logger);
    spdlog::flush_on(spdlog::level::trace);
}
}  // namespace ob_lidar
