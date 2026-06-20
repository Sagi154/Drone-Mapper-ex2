#pragma once

#include <drone_mapper/types/MissionTypes.h>

#include <optional>
#include <vector>

namespace drone_mapper::io {

template <typename T>
struct ConfigParseResult {
    bool ok = false;
    T value{};
    std::vector<types::ErrorRef> errors{};
};

} // namespace drone_mapper::io
