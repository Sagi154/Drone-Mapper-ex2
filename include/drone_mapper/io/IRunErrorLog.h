#pragma once

#include <drone_mapper/types/MissionTypes.h>

namespace drone_mapper::io {

class IRunErrorLog {
public:
    virtual ~IRunErrorLog() = default;

    virtual void log(const types::ErrorRef& error) = 0;
};

} // namespace drone_mapper::io
