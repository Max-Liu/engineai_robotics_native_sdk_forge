#!/bin/bash

set -e

SCRIPT_DIR="$(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)"
if [ -d "$SCRIPT_DIR/simulation" ]; then
    ROOT_DIR="$SCRIPT_DIR"
else
    ROOT_DIR="$(dirname "$SCRIPT_DIR")"
fi

export ENGINEAI_ROBOTICS_DIR="$ROOT_DIR"
export ENGINEAI_ROBOTICS_ASSETS="$ENGINEAI_ROBOTICS_DIR/assets"
export ENGINEAI_ROBOTICS_THIRD_PARTY="/opt/engineai_robotics_third_party"
export ENGINEAI_ROBOTICS_HARDWARE="/opt/engineai_robotics_hardware"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$ENGINEAI_ROBOTICS_THIRD_PARTY/lib:$ENGINEAI_ROBOTICS_HARDWARE/lib:$ENGINEAI_ROBOTICS_DIR/core/lib:/opt/ros/humble/lib"

PYTHON_BIN="${PYTHON:-python3}"
exec "$PYTHON_BIN" "$ROOT_DIR/scripts/replay_npz_motion_in_mujoco.py" --sdk-root "$ROOT_DIR" "$@"
