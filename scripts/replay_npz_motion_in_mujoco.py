#!/usr/bin/env python3
"""Publish IsaacLab PM01 NPZ motion frames to the MuJoCo replay-state channel."""

from __future__ import annotations

import argparse
import os
import signal
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np


NUM_PM01_JOINTS = 24
DEFAULT_ROBOT = "pm01_edu"
DEFAULT_CHANNEL = "sim_replay_state"
DEFAULT_LCM_URL = "udpm://239.255.76.67:7667?ttl=0"

REQUIRED_KEYS = (
    "joint_pos",
    "joint_vel",
    "root_pos",
    "root_rot",
    "root_lin_vel",
    "root_ang_vel",
    "fps",
)


@dataclass(frozen=True)
class MotionData:
    path: Path
    joint_pos: np.ndarray
    joint_vel: np.ndarray
    root_pos: np.ndarray
    root_rot: np.ndarray
    root_lin_vel: np.ndarray
    root_ang_vel: np.ndarray
    fps: float

    @property
    def frame_count(self) -> int:
        return int(self.joint_pos.shape[0])


class SimStateMessage:
    """Small Python equivalent of core/include/data/lcm_data/SimState.hpp."""

    __slots__ = (
        "timestamp",
        "num_ranges",
        "joint_position",
        "joint_velocity",
        "joint_torque",
        "base_link_position",
        "base_link_linear_velocity",
        "base_link_quaternion",
        "base_link_angular_velocity",
        "imu_link_position",
        "imu_link_linear_velocity",
        "imu_link_quaternion",
        "imu_link_angular_velocity",
        "imu_sensor_quaternion",
        "imu_sensor_linear_acceleration",
        "imu_sensor_angular_velocity",
        "num_contact_ranges",
        "contact_force",
    )

    def __init__(self) -> None:
        self.timestamp = 0.0
        self.num_ranges = 0
        self.joint_position = np.zeros(0, dtype=np.float64)
        self.joint_velocity = np.zeros(0, dtype=np.float64)
        self.joint_torque = np.zeros(0, dtype=np.float64)
        self.base_link_position = np.zeros(3, dtype=np.float64)
        self.base_link_linear_velocity = np.zeros(3, dtype=np.float64)
        self.base_link_quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.base_link_angular_velocity = np.zeros(3, dtype=np.float64)
        self.imu_link_position = np.zeros(3, dtype=np.float64)
        self.imu_link_linear_velocity = np.zeros(3, dtype=np.float64)
        self.imu_link_quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.imu_link_angular_velocity = np.zeros(3, dtype=np.float64)
        self.imu_sensor_quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.imu_sensor_linear_acceleration = np.zeros(3, dtype=np.float64)
        self.imu_sensor_angular_velocity = np.zeros(3, dtype=np.float64)
        self.num_contact_ranges = 0
        self.contact_force = np.zeros(0, dtype=np.float64)

    @staticmethod
    def _fingerprint() -> int:
        base_hash = 0x56DA9E44F01AF82E
        return (((base_hash << 1) & 0xFFFFFFFFFFFFFFFF) + (base_hash >> 63)) & 0xFFFFFFFFFFFFFFFF

    def encode(self) -> bytes:
        chunks = [
            struct.pack(">Q", self._fingerprint()),
            struct.pack(">d", float(self.timestamp)),
            struct.pack(">i", int(self.num_ranges)),
            _encode_float64_array(self.joint_position, self.num_ranges),
            _encode_float64_array(self.joint_velocity, self.num_ranges),
            _encode_float64_array(self.joint_torque, self.num_ranges),
            _encode_float64_array(self.base_link_position, 3),
            _encode_float64_array(self.base_link_linear_velocity, 3),
            _encode_float64_array(self.base_link_quaternion, 4),
            _encode_float64_array(self.base_link_angular_velocity, 3),
            _encode_float64_array(self.imu_link_position, 3),
            _encode_float64_array(self.imu_link_linear_velocity, 3),
            _encode_float64_array(self.imu_link_quaternion, 4),
            _encode_float64_array(self.imu_link_angular_velocity, 3),
            _encode_float64_array(self.imu_sensor_quaternion, 4),
            _encode_float64_array(self.imu_sensor_linear_acceleration, 3),
            _encode_float64_array(self.imu_sensor_angular_velocity, 3),
            struct.pack(">i", int(self.num_contact_ranges)),
            _encode_float64_array(self.contact_force, self.num_contact_ranges),
        ]
        return b"".join(chunks)


def _encode_float64_array(values: np.ndarray, expected_size: int) -> bytes:
    array = np.asarray(values, dtype=np.float64).reshape(-1)
    if array.size != expected_size:
        raise ValueError(f"Expected {expected_size} double values, got {array.size}.")
    return array.astype(">f8", copy=False).tobytes()


def _resolve_motion_path(path_arg: str) -> Path:
    path_text = path_arg[1:] if path_arg.startswith("@") else path_arg
    path = Path(os.path.expanduser(path_text))
    if path.is_file():
        return path.resolve()
    raise FileNotFoundError(f"NPZ file not found: {path_arg}")


def _read_npz_array(data: np.lib.npyio.NpzFile, key: str) -> np.ndarray:
    return np.asarray(data[key], dtype=np.float64)


def load_motion(path_arg: str) -> MotionData:
    path = _resolve_motion_path(path_arg)
    with np.load(path) as data:
        missing = [key for key in REQUIRED_KEYS if key not in data.files]
        if missing:
            available = ", ".join(sorted(data.files))
            raise ValueError(f"Missing required NPZ key(s): {', '.join(missing)}. Available keys: {available}")

        motion = MotionData(
            path=path,
            joint_pos=_read_npz_array(data, "joint_pos"),
            joint_vel=_read_npz_array(data, "joint_vel"),
            root_pos=_read_npz_array(data, "root_pos"),
            root_rot=_read_npz_array(data, "root_rot"),
            root_lin_vel=_read_npz_array(data, "root_lin_vel"),
            root_ang_vel=_read_npz_array(data, "root_ang_vel"),
            fps=float(np.asarray(data["fps"]).item()),
        )

    validate_motion(motion)
    return motion


def validate_motion(motion: MotionData) -> None:
    expected_shapes = {
        "joint_pos": (None, NUM_PM01_JOINTS),
        "joint_vel": (None, NUM_PM01_JOINTS),
        "root_pos": (None, 3),
        "root_rot": (None, 4),
        "root_lin_vel": (None, 3),
        "root_ang_vel": (None, 3),
    }
    arrays = {
        "joint_pos": motion.joint_pos,
        "joint_vel": motion.joint_vel,
        "root_pos": motion.root_pos,
        "root_rot": motion.root_rot,
        "root_lin_vel": motion.root_lin_vel,
        "root_ang_vel": motion.root_ang_vel,
    }

    frame_count = None
    for key, array in arrays.items():
        expected = expected_shapes[key]
        if array.ndim != 2:
            raise ValueError(f"{key} must be 2-D, got shape {array.shape}.")
        if array.shape[1] != expected[1]:
            raise ValueError(f"{key} must have shape (T, {expected[1]}), got {array.shape}.")
        if frame_count is None:
            frame_count = int(array.shape[0])
        elif array.shape[0] != frame_count:
            raise ValueError(f"{key} has {array.shape[0]} frames, expected {frame_count}.")
        if not np.isfinite(array).all():
            raise ValueError(f"{key} contains non-finite values.")

    if frame_count is None or frame_count <= 0:
        raise ValueError("Motion must contain at least one frame.")
    if not np.isfinite(motion.fps) or motion.fps <= 0.0:
        raise ValueError(f"fps must be a positive scalar, got {motion.fps}.")

    quat_norms = np.linalg.norm(motion.root_rot, axis=1)
    if not np.isfinite(quat_norms).all() or np.any(quat_norms <= 1e-9):
        raise ValueError("root_rot contains an invalid zero-length quaternion.")


def _normalize_quaternion_wxyz(quaternion: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-9:
        raise ValueError("Cannot normalize a zero-length quaternion.")
    return np.asarray(quaternion, dtype=np.float64) / norm


def make_message(motion: MotionData, frame_index: int) -> SimStateMessage:
    msg = SimStateMessage()
    root_quat = _normalize_quaternion_wxyz(motion.root_rot[frame_index])

    msg.timestamp = frame_index / motion.fps
    msg.num_ranges = NUM_PM01_JOINTS
    msg.joint_position = motion.joint_pos[frame_index]
    msg.joint_velocity = motion.joint_vel[frame_index]
    msg.joint_torque = np.zeros(NUM_PM01_JOINTS, dtype=np.float64)
    msg.base_link_position = motion.root_pos[frame_index]
    msg.base_link_linear_velocity = motion.root_lin_vel[frame_index]
    msg.base_link_quaternion = root_quat
    msg.base_link_angular_velocity = motion.root_ang_vel[frame_index]
    msg.imu_link_position = motion.root_pos[frame_index]
    msg.imu_link_linear_velocity = motion.root_lin_vel[frame_index]
    msg.imu_link_quaternion = root_quat
    msg.imu_link_angular_velocity = motion.root_ang_vel[frame_index]
    msg.imu_sensor_quaternion = root_quat
    msg.imu_sensor_linear_acceleration = np.zeros(3, dtype=np.float64)
    msg.imu_sensor_angular_velocity = motion.root_ang_vel[frame_index]
    msg.num_contact_ranges = 0
    msg.contact_force = np.zeros(0, dtype=np.float64)
    return msg


def _parse_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _parse_lcm_config(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    if not path.is_file():
        return values

    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        values[key.strip()] = value.strip().strip("\"'")
    return values


def resolve_lcm_url(sdk_root: Path, robot: str, override: str | None) -> str:
    if override:
        return override

    config_path = sdk_root / "assets" / "config" / robot / "lcm" / "default.yaml"
    values = _parse_lcm_config(config_path)
    ip_port = values.get("ip_port")
    if not ip_port:
        return DEFAULT_LCM_URL

    multicast = _parse_bool(values.get("multicast", "false"))
    ttl = int(values.get("ttl", "1")) if multicast else 0
    return f"udpm://{ip_port}?ttl={ttl}"


def print_motion_summary(motion: MotionData, lcm_url: str, channel: str) -> None:
    duration = motion.frame_count / motion.fps
    print(f"[INFO] NPZ: {motion.path}")
    print(f"[INFO] Frames: {motion.frame_count}")
    print(f"[INFO] FPS: {motion.fps:g}")
    print(f"[INFO] Duration: {duration:.3f}s")
    print(f"[INFO] LCM URL: {lcm_url}")
    print(f"[INFO] Channel: {channel}")


def replay_motion(motion: MotionData, lcm_url: str, channel: str, loop: bool) -> None:
    try:
        import lcm
    except ImportError as exc:
        raise RuntimeError("Python package 'lcm' is required for publishing. Run inside engineai_robotics_env.") from exc

    lcm_handle = lcm.LCM(lcm_url)
    frame_period = 1.0 / motion.fps
    stop_requested = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    print("[INFO] Publishing replay states. Press Ctrl-C to stop.")
    published = 0
    try:
        while True:
            start_time = time.monotonic()
            for frame_index in range(motion.frame_count):
                if stop_requested:
                    print(f"\n[INFO] Stopped after publishing {published} frame(s).")
                    return

                msg = make_message(motion, frame_index)
                lcm_handle.publish(channel, msg.encode())
                published += 1

                target_time = start_time + (frame_index + 1) * frame_period
                sleep_s = target_time - time.monotonic()
                if sleep_s > 0.0:
                    time.sleep(sleep_s)

            if not loop:
                print(f"[INFO] Replay complete: published {published} frame(s).")
                return
    finally:
        if hasattr(lcm_handle, "close"):
            lcm_handle.close()


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Replay an IsaacLab/PM01 motion NPZ in SDK MuJoCo through sim_replay_state."
    )
    parser.add_argument(
        "motion_npz",
        help="Path to an IsaacLab PM01 training-format NPZ, e.g. ../data/motion_retarget/Jumping.npz.",
    )
    parser.add_argument("--loop", action="store_true", help="Loop the motion until interrupted.")
    parser.add_argument("--dry-run", action="store_true", help="Validate the NPZ and encode one frame without publishing.")
    parser.add_argument(
        "--robot",
        default=DEFAULT_ROBOT,
        help=f"Robot config used to read LCM settings. Default: {DEFAULT_ROBOT}.",
    )
    parser.add_argument("--sdk-root", default=None, help="SDK root. Defaults to the parent of this script directory.")
    parser.add_argument("--lcm-url", default=None, help="Override the LCM URL, e.g. udpm://239.255.76.67:7667?ttl=0.")
    parser.add_argument("--channel", default=DEFAULT_CHANNEL, help=f"LCM channel. Default: {DEFAULT_CHANNEL}.")
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    script_path = Path(__file__).resolve()
    sdk_root = Path(args.sdk_root).resolve() if args.sdk_root else script_path.parent.parent
    motion = load_motion(args.motion_npz)
    lcm_url = resolve_lcm_url(sdk_root, args.robot, args.lcm_url)

    print_motion_summary(motion, lcm_url, args.channel)
    first_message_size = len(make_message(motion, 0).encode())
    print(f"[INFO] Encoded SimState size: {first_message_size} bytes")

    if args.dry_run:
        print("[INFO] Dry run complete: NPZ format is valid.")
        return 0

    replay_motion(motion, lcm_url, args.channel, args.loop)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        raise SystemExit(1)
