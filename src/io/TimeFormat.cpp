#include <drone_mapper/io/TimeFormat.h>

#include <chrono>
#include <iomanip>
#include <sstream>

namespace drone_mapper::io {

std::string currentUtcTimestamp() {
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

} // namespace drone_mapper::io
