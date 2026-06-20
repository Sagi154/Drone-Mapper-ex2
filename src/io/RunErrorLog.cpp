#include <drone_mapper/io/RunErrorLog.h>

#include <chrono>
#include <iomanip>
#include <sstream>
#include <utility>

namespace drone_mapper::io {

namespace {

[[nodiscard]] std::string currentUtcTimestamp() {
    const auto now = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(now);
    std::tm utc{};
#if defined(_WIN32)
    gmtime_s(&utc, &time);
#else
    gmtime_r(&time, &utc);
#endif
    std::ostringstream formatted;
    formatted << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
    return formatted.str();
}

} // namespace

RunErrorLog::RunErrorLog(std::filesystem::path log_path)
    : log_path_(std::move(log_path)) {
    if (log_path_.has_parent_path()) {
        std::filesystem::create_directories(log_path_.parent_path());
    }
    stream_.open(log_path_, std::ios::app);
}

void RunErrorLog::log(const types::ErrorRef& error) {
    if (!stream_.is_open()) {
        stream_.open(log_path_, std::ios::app);
    }
    stream_ << currentUtcTimestamp() << ' ' << error.code << ' ' << error.message << '\n';
    stream_.flush();
}

} // namespace drone_mapper::io
