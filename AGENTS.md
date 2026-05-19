# Repository Guidelines

## Project Structure & Module Organization

This repository is a CMake-based C++20 robotics SDK with shell and Python tools. Main code lives in `src/`: `executor/` starts the runtime, `runner/` contains motion/control plugins, `hardware/` contains drivers, `ros2_node/` contains ROS2 bridge nodes, `protocol/interface_protocol/` defines messages and services, and `data/_param/` stores parameters. Robot configs and models are under `assets/config/<robot>/` and `assets/resource/`. MuJoCo code is in `simulation/mujoco/`; scripts are in `scripts/`; Docker setup is in `docker/`; the PyQt virtual gamepad is in `tools/virtual_gamepad/`. Generated output belongs in `build/`.

## Build, Test, and Development Commands

Run all compile, test, and runtime commands inside Docker: first run `engineai_robotics_env`, then execute the command.

- `./docker/generate.sh`: create the container development environment and `engineai_robotics_env` shortcut.
- `engineai_robotics_env`: enter the container before building, testing, or running.
- `./build.sh`: build and install the SDK into `build/_install`.
- `./build.sh -t debug -T`: debug build with GoogleTest targets enabled.
- `./build.sh -m rl_walking_example`: rebuild one runner target after a full build.
- `ctest --test-dir build --output-on-failure`: run registered tests after building with `-T`.
- `./run.sh [pm01_edu|t800]`: run the installed executor for the default or named robot.
- `./scripts/build_mujoco.sh [--mirror-deps]`: build the MuJoCo simulator.
- `python3 -m pip install -r tools/virtual_gamepad/requirements.txt`: install gamepad UI dependencies.

## Coding Style & Naming Conventions

Use C++20, two-space indentation, and `.cc`/`.h` file extensions. Keep namespaces and target names aligned with directory structure; runner modules use snake_case directories such as `rl_stamp_tracking`. Prefer existing CMake helpers (`get_namespace`, `add_googletest`) over custom target wiring. Python tooling follows standard PEP 8-style four-space indentation.

## Testing Guidelines

Tests use GoogleTest and are compiled only when `BUILD_TESTS` is enabled, typically through `./build.sh -T`. Place tests near the module under `src/.../src/tests/` or as `test_*.cc`/`test_*.cpp` files so CMake discovery includes them. Avoid tests that require live robot hardware unless clearly documented.

## Commit & Pull Request Guidelines

Recent history uses short imperative subjects, often with a Conventional Commit prefix such as `feat:`. Prefer messages like `feat: add rl_stamp_tracking mode` or `Update readme about ros interface`. Pull requests should describe behavior changes, list affected robot models/configs, include build/test results, and link related issues. Add screenshots only for UI changes such as `tools/virtual_gamepad/` or documentation images.

## Safety & Configuration Notes

Do not commit local secrets, generated crash dumps, or machine-specific device permissions. Treat real-robot scripts such as `install.sh`, `scripts/run_robot.sh`, and `scripts/set_imu_tty.sh` as deployment-sensitive; document required hardware and operator assumptions in any change that touches them.
