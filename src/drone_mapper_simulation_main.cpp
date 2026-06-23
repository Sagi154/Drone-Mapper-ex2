#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>
#include <drone_mapper/io/SimulationCli.h>
#include <drone_mapper/io/StderrErrorLog.h>
#include <drone_mapper/io/YamlConfigParsers.h>

#include <iostream>
#include <memory>

int main(int argc, char** argv) {
    const drone_mapper::io::SimulationCliArgs args = drone_mapper::io::parseSimulationCliArgs(argc, argv);

    drone_mapper::io::StderrErrorLog log;
    const drone_mapper::io::ConfigParseResult<drone_mapper::types::SimulationCompositionData> composition_result =
        drone_mapper::io::parseCompositionFile(args.composition_file, log);
    if (!composition_result.ok) {
        for (const drone_mapper::types::ErrorRef& error : composition_result.errors) {
            log.log(error);
        }
        return 1;
    }

    auto run_factory = std::make_unique<drone_mapper::SimulationRunFactoryImpl>();
    drone_mapper::SimulationManager simulation{std::move(run_factory)};

    const drone_mapper::types::SimulationManagerReport report =
        simulation.run(composition_result.value, args.output_path);

    std::cout << "Assignment 2 simulator skeleton ran "
              << report.runs.size()
              << " run(s).\n";
    return 0;
}
