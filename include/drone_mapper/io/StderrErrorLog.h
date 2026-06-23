#pragma once

#include <drone_mapper/io/IRunErrorLog.h>

namespace drone_mapper::io {

class StderrErrorLog final : public IRunErrorLog {
public:
    void log(const types::ErrorRef& error) override;
};

} // namespace drone_mapper::io
