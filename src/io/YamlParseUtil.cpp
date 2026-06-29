#include "YamlParseUtil.hpp"

#include <drone_mapper/types/MapTypes.h>

#include <fstream>
#include <sstream>

namespace drone_mapper::io::detail {

namespace {

[[nodiscard]] bool fileExists(const std::filesystem::path& path) {
    return std::filesystem::exists(path);
}

} // namespace

void logRecoverable(IRunErrorLog& log, const std::string& code, const std::string& message) {
    log.log({code, message});
}

std::optional<YAML::Node> loadYamlFile(const std::filesystem::path& path, IRunErrorLog& log,
                                         const std::string& context) {
    if (!fileExists(path)) {
        std::ostringstream message;
        message << context << " could not open file \"" << path.string() << "\" — using defaults";
        logRecoverable(log, "CONFIG_FILE_NOT_FOUND", message.str());
        return std::nullopt;
    }

    try {
        return YAML::LoadFile(path.string());
    } catch (const YAML::Exception& exception) {
        std::ostringstream message;
        message << context << " parse error in \"" << path.string() << "\": " << exception.what();
        logRecoverable(log, "CONFIG_PARSE_ERROR", message.str());
        return std::nullopt;
    }
}

YAML::Node configRoot(const YAML::Node& root, const char* wrapper_key) {
    if (root[wrapper_key]) {
        return root[wrapper_key];
    }
    return root;
}

std::optional<double> readScalarDouble(const YAML::Node& node, const char* key) {
    const YAML::Node child = node[key];
    if (!child || !child.IsScalar()) {
        return std::nullopt;
    }
    try {
        return child.as<double>();
    } catch (const YAML::Exception&) {
        return std::nullopt;
    }
}

std::optional<PhysicalLength> readLengthCm(const YAML::Node& node, const char* key) {
    const auto value = readScalarDouble(node, key);
    if (!value.has_value()) {
        return std::nullopt;
    }
    return *value * cm;
}

std::optional<HorizontalAngle> readHorizontalAngleDeg(const YAML::Node& node, const char* key) {
    const auto value = readScalarDouble(node, key);
    if (!value.has_value()) {
        return std::nullopt;
    }
    return *value * horizontal_angle[deg];
}

std::optional<Position3D> readPosition3D(const YAML::Node& node) {
    if (!node || !node.IsMap()) {
        return std::nullopt;
    }

    const auto x = readScalarDouble(node, "x_cm");
    const auto y = readScalarDouble(node, "y_cm");
    const auto height = readScalarDouble(node, "height_cm");
    if (!x.has_value() || !y.has_value() || !height.has_value()) {
        return std::nullopt;
    }

    Position3D position{};
    position.x = *x * x_extent[cm];
    position.y = *y * y_extent[cm];
    position.z = *height * z_extent[cm];
    return position;
}

std::optional<Position3D> readMapOffset(const YAML::Node& node) {
    if (!node || !node.IsMap()) {
        return std::nullopt;
    }

    Position3D offset{};
    if (const auto x = readScalarDouble(node, "x_cm")) {
        offset.x = *x * x_extent[cm];
    } else if (const auto x_offset = readScalarDouble(node, "x_offset")) {
        offset.x = *x_offset * x_extent[cm];
    } else {
        return std::nullopt;
    }

    if (const auto y = readScalarDouble(node, "y_cm")) {
        offset.y = *y * y_extent[cm];
    } else if (const auto y_offset = readScalarDouble(node, "y_offset")) {
        offset.y = *y_offset * y_extent[cm];
    } else {
        return std::nullopt;
    }

    if (const auto z = readScalarDouble(node, "z_cm")) {
        offset.z = *z * z_extent[cm];
    } else if (const auto height_offset = readScalarDouble(node, "height_offset")) {
        offset.z = *height_offset * z_extent[cm];
    } else {
        return std::nullopt;
    }

    return offset;
}

std::optional<types::MappingBounds> readMissionBoundaries(const YAML::Node& node) {
    const YAML::Node boundaries = node["boundaries"];
    if (!boundaries || !boundaries.IsMap()) {
        return std::nullopt;
    }

    types::MappingBounds bounds{};

    if (const YAML::Node axis = boundaries["x_boundary"]; axis && axis.IsMap()) {
        if (const auto value = readScalarDouble(axis, "min_cm")) {
            bounds.min_x = *value * x_extent[cm];
        }
        if (const auto value = readScalarDouble(axis, "max_cm")) {
            bounds.max_x = *value * x_extent[cm];
        }
    }
    if (const YAML::Node axis = boundaries["y_boundary"]; axis && axis.IsMap()) {
        if (const auto value = readScalarDouble(axis, "min_cm")) {
            bounds.min_y = *value * y_extent[cm];
        }
        if (const auto value = readScalarDouble(axis, "max_cm")) {
            bounds.max_y = *value * y_extent[cm];
        }
    }
    if (const YAML::Node axis = boundaries["height_boundary"]; axis && axis.IsMap()) {
        if (const auto value = readScalarDouble(axis, "min_cm")) {
            bounds.min_height = *value * z_extent[cm];
        }
        if (const auto value = readScalarDouble(axis, "max_cm")) {
            bounds.max_height = *value * z_extent[cm];
        }
    }
    return bounds;
}

} // namespace drone_mapper::io::detail
