#include <drone_mapper/io/SimulationCli.h>

namespace drone_mapper::io {

SimulationCliArgs parseSimulationCliArgs(int argc, char** argv) {
    SimulationCliArgs args;
    args.composition_file =
        (argc >= 2) ? std::filesystem::path{argv[1]} : std::filesystem::path{"simulation.yaml"};
    args.output_path =
        (argc >= 3) ? std::filesystem::path{argv[2]} : std::filesystem::current_path();
    return args;
}

} // namespace drone_mapper::io
