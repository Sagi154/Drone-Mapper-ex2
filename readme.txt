Drone-Mapper-ex2 — Assignment 2 (2026B)
Contributors: Sagi Eisenberg (207190406), Yoav Naaman (209543255)

================================================================================
BUILD
================================================================================

Prerequisites:
  - vcpkg (set VCPKG_ROOT to your vcpkg root, e.g. C:\Users\<you>\vcpkg)
  - CMake 3.15+
  - C++20 compiler (MSVC 2022 on Windows; GCC/Clang on Linux)

From the repo root:

  cmake --preset default
  cmake --build --preset default

Windows note: configure and build from a Visual Studio Developer shell so MSVC
is used (vcpkg x64-windows packages require it). Example (PowerShell):

  $env:VCPKG_ROOT = "C:\Users\<you>\vcpkg"
  Import-Module "<VS>\Common7\Tools\Microsoft.VisualStudio.DevShell.dll"
  Enter-VsDevShell -VsInstallPath "<VS>" -SkipAutomaticLocation -DevCmdArguments "-arch=amd64"
  cmake --preset default
  cmake --build --preset default

Targets:
  build/drone_mapper_simulation.exe   (or ./build/drone_mapper_simulation)
  build/maps_comparison.exe
  build/drone_mapper_simulation_test  (single umbrella binary; required GTest filter
    prefixes: SimulationManager.*, SimulationRun.*, MissionControl.*, DroneControl.*,
    MappingAlgorithm.*, MapsComparison.*, MockLidar.*, Integration.*)

Dependencies are declared in vcpkg.json (mp-units, yaml-cpp, tinynpy, gtest).

================================================================================
RUN
================================================================================

Simulation (nested composition → aligned pairs × drones × lidars):

  ./build/drone_mapper_simulation [<simulation.yaml>] [<output_path>]

  - Missing yaml arg  -> use simulation.yaml in the current working directory
  - Filename only     -> look in CWD
  - Relative path     -> resolved under CWD
  - Absolute path     -> used as given
  - Missing output_path -> use CWD

Maps comparison utility:

  ./build/maps_comparison <origin_map> <target_map> [comparison_config=<path>]

  - stdout: score only (0.0–100.0 float)
  - on error: stdout prints -1, descriptive message on stderr

================================================================================
ERROR HANDLING
================================================================================

Failures are handled at two levels: program startup (composition load) and
per-run execution (factory startup or mission loop).

Program startup failure
  - Trigger: composition YAML missing or unparseable
  - Behavior: errors logged to stderr via StderrErrorLog; main returns 1
  - Output: no simulation_output.yaml and no output_results/ are written

Per-run startup failure
  - Trigger: config_load_error on any simulation/mission/drone/lidar config, or
    MAP_FILE_NOT_FOUND when the hidden .npy map is missing or corrupt
  - Behavior: errors logged immediately to the run's error.log; mission_score
    is -1 in simulation_output.yaml; simulation continues with the next run
  - Output: run folder and error.log are still created

Per-run mission failure
  - Trigger: MissionRunStatus::Error during the step loop
  - Behavior: same as per-run startup failure (score -1, error.log written,
    simulation continues)

Corrupt .npy
  - Treated the same as a missing map file: MAP_FILE_NOT_FOUND, score -1,
    run continues

See OUTPUT LAYOUT below for run folder layout, error.log format, and failed
continuable scenario details.

================================================================================
OUTPUT LAYOUT
================================================================================

All artifacts are written under <output_path> (defaults to CWD).

<output_path>/
  simulation_output.yaml
  output_results/
    run_0001/
      output_map.npy
      error.log
    run_0002/
      output_map.npy
      error.log
    ...

Run folder naming
  - run_NNNN: zero-padded 4-digit, 1-based index (0001, 0002, …)
  - One folder per combination, in SimulationManager loop order:
      (simulation, mission) pairs × drones × lidars

Run index (1-based run_id)
  run_id = 1 + pair_i * (D * L)
              + drn_k * L
              + lid_m

  Where pair_i is the 0-based index into the aligned simulations[]/missions[] vectors
  (equal length; one entry per nested simulation+mission pair from composition YAML),
  and D, L are the sizes of drones and lidars respectively.

  Examples (2 pairs, 1 drone, 2 lidars):
    run_0001  pairs[0], drones[0], lidars[0]
    run_0002  pairs[0], drones[0], lidars[1]
    run_0003  pairs[1], drones[0], lidars[0]
    run_0004  pairs[1], drones[0], lidars[1]

Per-run files
  - output_map.npy  — final voxel map written by MissionControlImpl (NumPy format)
  - error.log       — all errors for this run; each line flushed immediately

error.log line format
  <ISO-8601 UTC timestamp> <ERROR_CODE> <user-facing message>

  Example:
    2026-06-20T14:32:01Z MAP_FILE_NOT_FOUND data_maps/missing.npy

Failed continuable scenarios
  - mission_score in simulation_output.yaml is -1
  - run folder and error.log are still created; simulation continues with next run
  - output_map.npy: save partial map if any mission steps ran; otherwise omit file

Path helpers (shared by factory and manager)
  runOutputDir(output_path, run_id)   -> output_results/run_NNNN/
  runOutputMap(output_path, run_id)   -> output_results/run_NNNN/output_map.npy
  runErrorLog(output_path, run_id)    -> output_results/run_NNNN/error.log

================================================================================
simulation_output.yaml FORMAT
================================================================================

Written by SimulationManager after all runs complete. Paths are relative to
<output_path> unless absolute paths are used.

score_report:
  generated_at_utc: "<ISO-8601 UTC timestamp>"
  metric: "maps_comparison_score"
  score_range: [0.0, 100.0]
  error_score: -1
  runs:
    - run_id: 1
      run_dir: "output_results/run_0001"
      output_map_file: "output_results/run_0001/output_map.npy"
      error_log_file: "output_results/run_0001/error.log"
      resolution_request_status: ACCEPTED | IGNORED | IGNORED TOO SMALL
      mission_score: <float, or -1 on failure>
      config_indices:
        simulation: 0
        mission: 0
        drone: 0
        lidar: 0
      simulation:
        map_filename: "<path>"
        map_resolution_cm: <float>
        map_offset: {x_cm, y_cm, z_cm}
        initial_drone_position: {x_cm, y_cm, z_cm}
        initial_angle_deg: <float>
      mission:
        max_steps: <int>
        gps_resolution_cm: <float>
        output_mapping_resolution_factor: <double>
      mission_results:
        - status: COMPLETED | MAX_STEPS | ERROR
          steps: <int>
          errors:
            - code: "<ERROR_CODE>"
              message: "<user-facing message>"

Mapping to code types
  - score_report     <- SimulationManagerReport
  - runs[]           <- SimulationManagerReport.runs (SimulationResult)
  - mission_results  <- SimulationResult.mission_results (MissionRunResult)
  - config_indices   <- pair index (same for simulation and mission) plus drone/lidar indices
  - run_id           <- 1-based; must match run_NNNN folder name
