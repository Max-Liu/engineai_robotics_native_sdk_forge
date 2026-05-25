#!/bin/bash

set -e

SCRIPT_DIR="$(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)"
if [ -d "$SCRIPT_DIR/simulation" ]; then
    ROOT_DIR="$SCRIPT_DIR"
else
    ROOT_DIR="$(dirname "$SCRIPT_DIR")"
fi

usage() {
  cat <<'EOF'
Usage: ./scripts/replay_mujoco_log.sh <log.mcap> [robot] [mujoco_log_replay options]

Examples:
  ./scripts/replay_mujoco_log.sh logs/rl_tracking_motion_kick_20260523_103906_18968.mcap pm01_edu --speed 1.0
  ./scripts/replay_mujoco_log.sh logs/rl_tracking_motion_kick_20260523_103906_18968.mcap pm01_edu --mode state --speed 1.0
  ./scripts/replay_mujoco_log.sh logs/rl_tracking_motion_kick_20260523_103906_18968.mcap pm01_edu --dry-run
EOF
}

if [ $# -lt 1 ]; then
  usage
  exit 1
fi

LOG_PATH="$1"
shift

ROBOT="pm01_edu"
if [ $# -gt 0 ] && [[ "$1" != --* ]]; then
  ROBOT="$1"
  shift
fi

if [[ "$LOG_PATH" != /* ]]; then
  LOG_PATH="$ROOT_DIR/$LOG_PATH"
fi
LOG_PATH="$(realpath -m "$LOG_PATH")"

export ENGINEAI_ROBOTICS_DIR="$ROOT_DIR"
export ENGINEAI_ROBOTICS_ASSETS="$ENGINEAI_ROBOTICS_DIR/assets"
export ENGINEAI_ROBOTICS_THIRD_PARTY="/opt/engineai_robotics_third_party"
export ENGINEAI_ROBOTICS_HARDWARE="/opt/engineai_robotics_hardware"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$ENGINEAI_ROBOTICS_THIRD_PARTY/lib:$ENGINEAI_ROBOTICS_HARDWARE/lib:$ENGINEAI_ROBOTICS_DIR/core/lib:$ENGINEAI_ROBOTICS_DIR/lib:/opt/ros/humble/lib"

BINARY="$ROOT_DIR/simulation/mujoco/build/mujoco_log_replay"
if [ ! -x "$BINARY" ]; then
  ALT_BINARY="$ROOT_DIR/simulation/mujoco/build/src/replay/mujoco_log_replay"
  if [ -x "$ALT_BINARY" ]; then
    BINARY="$ALT_BINARY"
  else
    echo "[ERROR] mujoco_log_replay not found. Run ./scripts/build_mujoco.sh first." >&2
    exit 1
  fi
fi

cd "$ROOT_DIR/simulation/mujoco/build"
exec "$BINARY" --log "$LOG_PATH" --robot "$ROBOT" "$@"
