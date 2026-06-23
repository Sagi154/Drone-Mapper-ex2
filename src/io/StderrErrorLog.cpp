#include <drone_mapper/io/StderrErrorLog.h>
#include <drone_mapper/io/TimeFormat.h>

#include <iostream>

namespace drone_mapper::io {

void StderrErrorLog::log(const types::ErrorRef& error) {
    std::cerr << currentUtcTimestamp() << ' ' << error.code << ' ' << error.message << '\n';
}

} // namespace drone_mapper::io
