#pragma once

#include <drone_mapper/io/IRunErrorLog.h>

#include <filesystem>
#include <fstream>

namespace drone_mapper::io {

class RunErrorLog final : public IRunErrorLog {
public:
    explicit RunErrorLog(std::filesystem::path log_path);

    void log(const types::ErrorRef& error) override;

private:
    std::filesystem::path log_path_;
    std::ofstream stream_;
};

} // namespace drone_mapper::io
