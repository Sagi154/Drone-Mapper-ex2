// maps_comparison_main.cpp
// Standalone map scoring utility. stdout = score only; errors print -1 on stdout.

#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MapsComparison.h>

#include <TinyNPY.h>

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

namespace {

[[nodiscard]] std::optional<types::MapConfig> readMapConfigNode(const YAML::Node& node) {
    if (!node || !node.IsMap()) {
        return std::nullopt;
    }

    types::MapConfig config{};

    if (const YAML::Node resolution = node["map_res_cm"]; resolution && resolution.IsScalar()) {
        config.resolution = resolution.as<double>() * cm;
    }

    if (const YAML::Node offset = node["map_offset"]; offset && offset.IsMap()) {
        const auto read_cm = [&](const char* key) -> double {
            const YAML::Node value = offset[key];
            return (value && value.IsScalar()) ? value.as<double>() : 0.0;
        };
        config.offset = Position3D{
            read_cm("x_cm") * x_extent[cm],
            read_cm("y_cm") * y_extent[cm],
            read_cm("z_cm") * z_extent[cm],
        };
    }

    if (const YAML::Node bounds = node["map_boundaries"]; bounds && bounds.IsMap()) {
        const auto read_cm = [&](const char* key) -> double {
            const YAML::Node value = bounds[key];
            return (value && value.IsScalar()) ? value.as<double>() : 0.0;
        };
        config.boundaries.min_x = read_cm("min_x_cm") * x_extent[cm];
        config.boundaries.max_x = read_cm("max_x_cm") * x_extent[cm];
        config.boundaries.min_y = read_cm("min_y_cm") * y_extent[cm];
        config.boundaries.max_y = read_cm("max_y_cm") * y_extent[cm];
        config.boundaries.min_height = read_cm("min_z_cm") * z_extent[cm];
        config.boundaries.max_height = read_cm("max_z_cm") * z_extent[cm];
    }

    return config;
}

[[nodiscard]] types::MapConfig defaultSharedMapConfig() {
    types::MapConfig config{};
    config.resolution = 10.0 * cm;
    config.offset = Position3D{};
    return config;
}

[[nodiscard]] std::optional<std::filesystem::path>
parseComparisonConfigArg(const std::string& arg) {
    constexpr std::string_view prefix = "comparison_config=";
    if (!arg.starts_with(prefix)) {
        return std::nullopt;
    }
    return std::filesystem::path{arg.substr(prefix.size())};
}

[[nodiscard]] std::optional<Map3DImpl> loadMapFromDisk(const std::filesystem::path& path,
                                                       const types::MapConfig& config) {
    auto map_array = std::make_shared<NpyArray>();
    const LPCSTR load_error = map_array->LoadNPY(path.string());
    if (load_error != nullptr) {
        return std::nullopt;
    }

    return Map3DImpl{std::move(map_array), config};
}

void printErrorAndExit(const std::string& message) {
    std::cout << "-1\n";
    std::cerr << message << '\n';
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 3 || argc > 4) {
        printErrorAndExit(
            "Usage: maps_comparison <origin_map> <target_map> [comparison_config=<path>]");
        return 1;
    }

    types::MapConfig origin_config = defaultSharedMapConfig();
    types::MapConfig target_config = defaultSharedMapConfig();

    if (argc == 4) {
        const std::optional<std::filesystem::path> config_path =
            parseComparisonConfigArg(argv[3]);
        if (!config_path) {
            printErrorAndExit("Optional third argument must be comparison_config=<path>");
            return 1;
        }

        try {
            const YAML::Node root = YAML::LoadFile(config_path->string());
            if (const std::optional<types::MapConfig> parsed = readMapConfigNode(root["original_map"])) {
                origin_config = *parsed;
            }
            if (const std::optional<types::MapConfig> parsed = readMapConfigNode(root["target_map"])) {
                target_config = *parsed;
            }
        } catch (const YAML::Exception& ex) {
            printErrorAndExit(std::string{"Failed to parse comparison config: "} + ex.what());
            return 1;
        }
    }

    const std::optional<Map3DImpl> origin =
        loadMapFromDisk(std::filesystem::path{argv[1]}, origin_config);
    if (!origin) {
        printErrorAndExit("Failed to load origin map: " + std::string{argv[1]});
        return 1;
    }

    const std::optional<Map3DImpl> target =
        loadMapFromDisk(std::filesystem::path{argv[2]}, target_config);
    if (!target) {
        printErrorAndExit("Failed to load target map: " + std::string{argv[2]});
        return 1;
    }

    const std::vector<double> scores =
        drone_mapper::MapsComparison::compare(*origin, {&(*target)});
    if (scores.empty()) {
        printErrorAndExit("Comparison produced no scores.");
        return 1;
    }

    std::cout << scores.front() << '\n';
    return 0;
}
