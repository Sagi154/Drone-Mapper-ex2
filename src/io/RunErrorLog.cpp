#include <drone_mapper/io/RunErrorLog.h>
#include <drone_mapper/io/TimeFormat.h>

#include <utility>

namespace drone_mapper::io {

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
