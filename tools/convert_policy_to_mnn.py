#!/usr/bin/env python3
"""Convert an ONNX policy model to MNN format.

The script prefers PyMNN's Python entry point:
MNN.tools.mnnconvert:main

If that import is unavailable, it falls back to the MNNConvert / mnnconvert
executable.
"""

from __future__ import annotations

import argparse
import csv
import importlib
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any


CONVERTER_NAMES = (
    "MNNConvert",
    "mnnconvert",
)

TRACKING_RUNNER_NAME = "rl_tracking_motion_runner"
EXPECTED_OBSERVATION_NAMES = [
    "command",
    "motion_anchor_pos_b",
    "motion_anchor_ori_b",
    "base_ang_vel",
    "joint_pos",
    "joint_vel",
    "actions",
    "projected_gravity",
    "joint_error",
    "motion_phase",
]
POLICY_FIELD_ORDER = [
    "policy_file",
    "policy_io_mcap_enabled",
    "policy_io_mcap_dir",
    "time_step_total",
    "joint_names",
    "joint_stiffness",
    "joint_damping",
    "default_joint_pos",
    "action_scale",
    "observation_names",
    "observation_history_lengths",
    "transition_duration_s",
    "loop_motion",
    "reset_observation_history_on_loop",
    "auto_transition",
    "align_reference_to_robot_anchor",
]
ONNX_METADATA_FIELDS = [
    "time_step_total",
    "joint_names",
    "joint_stiffness",
    "joint_damping",
    "default_joint_pos",
    "action_scale",
    "observation_names",
    "observation_history_lengths",
]

class ConverterImportError(RuntimeError):
    """Raised when PyMNN's Python converter entry point cannot be imported."""


class PolicySyncError(RuntimeError):
    """Raised when motion-policy synchronization input is invalid."""


class _QuotedString(str):
    """String marker for YAML fields where quoting improves readability."""


def build_converter_args(
    input_path: Path,
    output_path: Path,
    biz_code: str,
    fp16: bool,
) -> list[str]:
    args = [
        "-f",
        "ONNX",
        "--modelFile",
        str(input_path),
        "--MNNModel",
        str(output_path),
        "--bizCode",
        biz_code,
    ]
    if fp16:
        args.append("--fp16")
    return args


def run_pymnn_converter(converter_args: list[str]) -> int:
    """Run the converter through PyMNN's Python console entry point."""
    try:
        mnnconvert = importlib.import_module("MNN.tools.mnnconvert")
    except ModuleNotFoundError as exc:
        if exc.name == "MNN" or exc.name.startswith("MNN."):
            raise ConverterImportError(
                "PyMNN is not installed in this Python environment."
            ) from exc
        raise
    except ImportError as exc:
        raise ConverterImportError(f"PyMNN converter import failed: {exc}") from exc

    if not hasattr(mnnconvert, "main"):
        raise ConverterImportError("MNN.tools.mnnconvert does not expose main().")

    old_argv = sys.argv[:]
    try:
        sys.argv = ["mnnconvert", *converter_args]
        result = mnnconvert.main()
    except SystemExit as exc:
        if exc.code is None:
            return 0
        if isinstance(exc.code, int):
            return exc.code
        print(exc.code, file=sys.stderr)
        return 1
    finally:
        sys.argv = old_argv

    return 0 if result is None else int(result)


def find_converter(explicit_path: str | None) -> str | None:
    """Find the MNN converter executable."""
    if explicit_path:
        path = Path(explicit_path).expanduser()
        return str(path) if path.exists() else None

    env_path = os.environ.get("MNNCONVERT")
    if env_path:
        path = Path(env_path).expanduser()
        return str(path) if path.exists() else None

    for name in CONVERTER_NAMES:
        found = shutil.which(name)
        if found:
            return found

    local_candidates = (
        Path.cwd() / "MNNConvert",
        Path.cwd() / "mnnconvert",
        Path.cwd() / "build" / "MNNConvert",
        Path.cwd() / "build" / "tools" / "converter" / "MNNConvert",
    )
    for path in local_candidates:
        if path.exists():
            return str(path)

    return None


def split_metadata_list(value: str) -> list[str]:
    """Parse a comma-separated ONNX metadata value."""
    try:
        row = next(csv.reader([value], skipinitialspace=True))
    except csv.Error as exc:
        raise PolicySyncError(f"Invalid comma-separated metadata value: {value}") from exc
    return [item.strip() for item in row if item.strip()]


def parse_int_metadata(name: str, value: str) -> int:
    try:
        number = float(value)
    except ValueError as exc:
        raise PolicySyncError(f"Metadata {name} must be an integer, got {value!r}") from exc
    if not number.is_integer():
        raise PolicySyncError(f"Metadata {name} must be an integer, got {value!r}")
    integer = int(number)
    if integer <= 0:
        raise PolicySyncError(f"Metadata {name} must be positive, got {integer}")
    return integer


def parse_float_list_metadata(name: str, value: str) -> list[float]:
    items = split_metadata_list(value)
    if not items:
        raise PolicySyncError(f"Metadata {name} must not be empty")
    parsed: list[float] = []
    for item in items:
        try:
            parsed.append(float(item))
        except ValueError as exc:
            raise PolicySyncError(
                f"Metadata {name} contains a non-numeric value: {item!r}"
            ) from exc
    return parsed


def parse_int_list_metadata(name: str, value: str) -> list[int]:
    items = split_metadata_list(value)
    if not items:
        raise PolicySyncError(f"Metadata {name} must not be empty")
    parsed: list[int] = []
    for item in items:
        parsed.append(parse_int_metadata(name, item))
    return parsed


def normalize_joint_name(name: str) -> str:
    return name.strip().upper()


def observation_dim(name: str, num_actions: int) -> int:
    if name == "command":
        return 2 * num_actions
    if name == "motion_anchor_pos_b":
        return 3
    if name == "motion_anchor_ori_b":
        return 6
    if name == "base_ang_vel":
        return 3
    if name == "joint_pos":
        return num_actions
    if name == "joint_vel":
        return num_actions
    if name == "actions":
        return num_actions
    if name == "projected_gravity":
        return 3
    if name == "joint_error":
        return num_actions
    if name == "motion_phase":
        return 2
    raise PolicySyncError(f"Unsupported observation term: {name}")


def expected_observation_dim(
    observation_names: list[str],
    observation_history_lengths: list[int],
    num_actions: int,
) -> int:
    return sum(
        observation_dim(name, num_actions) * history_length
        for name, history_length in zip(observation_names, observation_history_lengths)
    )


def load_yaml_mapping(path: Path) -> dict[str, Any]:
    try:
        import yaml
    except ModuleNotFoundError as exc:
        raise PolicySyncError(
            "PyYAML is required for motion mode. Install it with: "
            "python3 -m pip install PyYAML"
        ) from exc

    try:
        with path.open("r", encoding="utf-8") as file:
            data = yaml.safe_load(file)
    except OSError as exc:
        raise PolicySyncError(f"Failed to read YAML file {path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise PolicySyncError(f"Invalid YAML in {path}: {exc}") from exc

    if not isinstance(data, dict):
        raise PolicySyncError(f"YAML file must contain a mapping: {path}")
    return data


def dump_yaml_mapping(data: dict[str, Any]) -> str:
    try:
        import yaml
    except ModuleNotFoundError as exc:
        raise PolicySyncError(
            "PyYAML is required for motion mode. Install it with: "
            "python3 -m pip install PyYAML"
        ) from exc

    class Dumper(yaml.SafeDumper):
        pass

    def quoted_string_representer(dumper: yaml.Dumper, value: _QuotedString) -> Any:
        return dumper.represent_scalar("tag:yaml.org,2002:str", str(value), style='"')

    Dumper.add_representer(_QuotedString, quoted_string_representer)
    return yaml.dump(
        data,
        Dumper=Dumper,
        allow_unicode=False,
        default_flow_style=False,
        sort_keys=False,
    )


def atomic_write_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            "w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as temp_file:
            temp_path = Path(temp_file.name)
            temp_file.write(content)
            temp_file.flush()
            os.fsync(temp_file.fileno())
        os.replace(temp_path, path)
    except OSError as exc:
        raise PolicySyncError(f"Failed to write {path}: {exc}") from exc
    finally:
        if temp_path and temp_path.exists():
            temp_path.unlink()


def find_tracking_param_tag(robot_config_dir: Path, motion_name: str) -> str:
    task_motion_path = robot_config_dir / "task_motion" / "default.yaml"
    if not task_motion_path.exists():
        raise PolicySyncError(f"Task motion YAML not found: {task_motion_path}")

    task_motion = load_yaml_mapping(task_motion_path)
    tasks = task_motion.get("tasks")
    if not isinstance(tasks, list):
        raise PolicySyncError(f"{task_motion_path} must contain a tasks list")

    motion_task: dict[str, Any] | None = None
    for task in tasks:
        if isinstance(task, dict) and task.get("motion") == motion_name:
            motion_task = task
            break

    if motion_task is None:
        raise PolicySyncError(
            f"Motion {motion_name!r} was not found in {task_motion_path}"
        )

    runners = motion_task.get("runner")
    if not isinstance(runners, list):
        raise PolicySyncError(
            f"Motion {motion_name!r} must contain a runner list in {task_motion_path}"
        )

    tracking_runners = [
        runner
        for runner in runners
        if isinstance(runner, dict)
        and runner.get("name") == TRACKING_RUNNER_NAME
        and runner.get("enabled", True)
    ]
    if not tracking_runners:
        runner_names = [
            str(runner.get("name"))
            for runner in runners
            if isinstance(runner, dict) and runner.get("name")
        ]
        raise PolicySyncError(
            f"Motion {motion_name!r} does not use enabled {TRACKING_RUNNER_NAME}; "
            f"found runners: {', '.join(runner_names) or '<none>'}"
        )
    if len(tracking_runners) > 1:
        raise PolicySyncError(
            f"Motion {motion_name!r} has multiple enabled {TRACKING_RUNNER_NAME} entries"
        )

    param_tag = tracking_runners[0].get("param_tag")
    if not isinstance(param_tag, str) or not param_tag:
        raise PolicySyncError(
            f"Motion {motion_name!r} {TRACKING_RUNNER_NAME} entry must set param_tag"
        )
    return param_tag


def validate_mode_scope(robot_config_dir: Path, param_tag: str) -> None:
    mode_path = robot_config_dir / "mode.yaml"
    if not mode_path.exists():
        raise PolicySyncError(f"Mode YAML not found: {mode_path}")

    mode_yaml = load_yaml_mapping(mode_path)
    modes = mode_yaml.get("mode")
    if not isinstance(modes, dict):
        raise PolicySyncError(f"{mode_path} must contain a mode mapping")

    expected_scope = f"{param_tag}/default"
    seen: list[str] = []
    wrong_scopes: list[str] = []
    for mode_name, entries in modes.items():
        if not isinstance(entries, list):
            continue
        for entry in entries:
            if not isinstance(entry, dict) or entry.get("tag") != param_tag:
                continue
            seen.append(str(mode_name))
            scope = entry.get("scope")
            if scope != expected_scope:
                wrong_scopes.append(
                    f"{mode_name}: expected {expected_scope}, got {scope!r}"
                )

    if not seen:
        raise PolicySyncError(f"mode.yaml does not map param_tag {param_tag!r}")
    if wrong_scopes:
        raise PolicySyncError(
            f"mode.yaml maps param_tag {param_tag!r} to the wrong scope: "
            + "; ".join(wrong_scopes)
        )


def get_value_shape(value_info: Any) -> list[int]:
    tensor_type = value_info.type.tensor_type
    dims: list[int] = []
    for dim in tensor_type.shape.dim:
        if dim.dim_value <= 0:
            dim_name = dim.dim_param or "<unknown>"
            raise PolicySyncError(
                f"ONNX tensor {value_info.name!r} has dynamic or unknown dimension "
                f"{dim_name!r}; static dimensions are required"
            )
        dims.append(dim.dim_value)
    if not dims:
        raise PolicySyncError(f"ONNX tensor {value_info.name!r} has no shape")
    return dims


def validate_tensor_shape(
    tensors: dict[str, Any],
    name: str,
    expected_shape: list[int],
    tensor_kind: str,
) -> None:
    value_info = tensors.get(name)
    if value_info is None:
        raise PolicySyncError(f"ONNX {tensor_kind} tensor is missing: {name}")
    shape = get_value_shape(value_info)
    if shape != expected_shape:
        raise PolicySyncError(
            f"ONNX {tensor_kind} tensor {name!r} has shape {shape}, "
            f"expected {expected_shape}"
        )


def load_tracking_policy_metadata(onnx_path: Path) -> dict[str, Any]:
    try:
        import onnx
    except ModuleNotFoundError as exc:
        raise PolicySyncError(
            "onnx is required for motion mode. Install it with: "
            "python3 -m pip install onnx"
        ) from exc

    try:
        model = onnx.load(str(onnx_path))
    except Exception as exc:
        raise PolicySyncError(f"Failed to load ONNX model {onnx_path}: {exc}") from exc

    metadata = {entry.key: entry.value for entry in model.metadata_props}
    missing = [name for name in ONNX_METADATA_FIELDS if name not in metadata]
    if missing:
        raise PolicySyncError(
            f"ONNX metadata is missing required fields: {', '.join(missing)}"
        )

    joint_names = [normalize_joint_name(name) for name in split_metadata_list(metadata["joint_names"])]
    if not joint_names:
        raise PolicySyncError("Metadata joint_names must not be empty")
    num_actions = len(joint_names)

    joint_stiffness = parse_float_list_metadata(
        "joint_stiffness", metadata["joint_stiffness"]
    )
    joint_damping = parse_float_list_metadata("joint_damping", metadata["joint_damping"])
    default_joint_pos = parse_float_list_metadata(
        "default_joint_pos", metadata["default_joint_pos"]
    )
    action_scale = parse_float_list_metadata("action_scale", metadata["action_scale"])

    joint_array_lengths = {
        "joint_stiffness": len(joint_stiffness),
        "joint_damping": len(joint_damping),
        "default_joint_pos": len(default_joint_pos),
        "action_scale": len(action_scale),
    }
    mismatched = [
        f"{name}={length}"
        for name, length in joint_array_lengths.items()
        if length != num_actions
    ]
    if mismatched:
        raise PolicySyncError(
            f"Joint metadata lengths must match joint_names={num_actions}; "
            + ", ".join(mismatched)
        )

    observation_names = split_metadata_list(metadata["observation_names"])
    if observation_names != EXPECTED_OBSERVATION_NAMES:
        raise PolicySyncError(
            "Metadata observation_names must match the rl_tracking_motion_runner "
            "layout exactly: "
            + ", ".join(EXPECTED_OBSERVATION_NAMES)
        )

    observation_history_lengths = parse_int_list_metadata(
        "observation_history_lengths", metadata["observation_history_lengths"]
    )
    if len(observation_history_lengths) != len(observation_names):
        raise PolicySyncError(
            "observation_history_lengths length must match observation_names length"
        )

    time_step_total = parse_int_metadata("time_step_total", metadata["time_step_total"])
    obs_dim = expected_observation_dim(
        observation_names, observation_history_lengths, num_actions
    )

    inputs = {value_info.name: value_info for value_info in model.graph.input}
    outputs = {value_info.name: value_info for value_info in model.graph.output}
    expected_input_shapes = {
        "obs": [1, obs_dim],
        "time_step": [1, 1],
    }
    expected_output_shapes = {
        "actions": [1, num_actions],
        "joint_pos": [1, num_actions],
        "joint_vel": [1, num_actions],
        "body_pos_w": [1, 3, 3],
        "body_quat_w": [1, 3, 4],
        "body_lin_vel_w": [1, 3, 3],
        "body_ang_vel_w": [1, 3, 3],
    }
    for name, shape in expected_input_shapes.items():
        validate_tensor_shape(inputs, name, shape, "input")
    for name, shape in expected_output_shapes.items():
        validate_tensor_shape(outputs, name, shape, "output")

    return {
        "time_step_total": time_step_total,
        "joint_names": joint_names,
        "joint_stiffness": joint_stiffness,
        "joint_damping": joint_damping,
        "default_joint_pos": default_joint_pos,
        "action_scale": action_scale,
        "observation_names": observation_names,
        "observation_history_lengths": observation_history_lengths,
    }


def build_tracking_yaml(
    existing_config: dict[str, Any],
    param_tag: str,
    metadata_config: dict[str, Any],
) -> dict[str, Any]:
    updated_values: dict[str, Any] = {
        "policy_file": _QuotedString(f"{param_tag}/policies/policy.mnn"),
        **metadata_config,
    }

    result: dict[str, Any] = {}
    for key in POLICY_FIELD_ORDER:
        if key in updated_values:
            result[key] = updated_values[key]
        elif key in existing_config:
            result[key] = existing_config[key]

    for key, value in existing_config.items():
        if key not in result:
            result[key] = value

    return result


def resolve_motion_paths(
    robot: str,
    motion: str,
    repo_root: Path,
) -> tuple[str, Path, Path, Path]:
    robot_config_dir = repo_root / "assets" / "config" / robot
    if not robot_config_dir.exists():
        raise PolicySyncError(f"Robot config directory not found: {robot_config_dir}")

    param_tag = find_tracking_param_tag(robot_config_dir, motion)
    validate_mode_scope(robot_config_dir, param_tag)

    param_dir = robot_config_dir / param_tag
    default_yaml_path = param_dir / "default.yaml"
    policy_onnx_path = param_dir / "policies" / "policy.onnx"
    policy_mnn_path = param_dir / "policies" / "policy.mnn"

    if not default_yaml_path.exists():
        raise PolicySyncError(f"Tracking config YAML not found: {default_yaml_path}")
    if not policy_onnx_path.exists():
        raise PolicySyncError(f"Policy ONNX file not found: {policy_onnx_path}")

    return param_tag, default_yaml_path, policy_onnx_path, policy_mnn_path


def convert_onnx_to_mnn(
    input_path: Path,
    output_path: Path,
    converter: str | None,
    biz_code: str,
    fp16: bool,
) -> int:
    converter_args = build_converter_args(
        input_path=input_path,
        output_path=output_path,
        biz_code=biz_code,
        fp16=fp16,
    )

    if not converter:
        try:
            print("Running: PyMNN MNN.tools.mnnconvert", flush=True)
            return_code = run_pymnn_converter(converter_args)
            if return_code != 0:
                print(
                    f"PyMNN converter failed with exit code {return_code}",
                    file=sys.stderr,
                )
                return return_code
            print(f"Converted: {output_path}")
            return 0
        except ConverterImportError as exc:
            print(f"PyMNN converter is unavailable: {exc}", file=sys.stderr)

    converter_path = find_converter(converter)
    if not converter_path:
        print(
            "No MNN converter was found.\n"
            "Install PyMNN with: python3 -m pip install -U MNN\n"
            "Or build MNN and pass --converter /path/to/MNNConvert.",
            file=sys.stderr,
        )
        return 127

    command = [converter_path, *converter_args]

    print("Running:", " ".join(command))
    completed = subprocess.run(command, check=False)
    if completed.returncode != 0:
        print(f"MNNConvert failed with exit code {completed.returncode}", file=sys.stderr)
        return completed.returncode

    print(f"Converted: {output_path}")
    return 0


def run_direct_mode(args: argparse.Namespace) -> int:
    input_path = Path(args.input or "policy.onnx").expanduser().resolve()
    output_path = Path(args.output or "policy.mnn").expanduser().resolve()

    if not input_path.exists():
        print(f"Input file not found: {input_path}", file=sys.stderr)
        return 2

    output_path.parent.mkdir(parents=True, exist_ok=True)
    return convert_onnx_to_mnn(
        input_path=input_path,
        output_path=output_path,
        converter=args.converter,
        biz_code=args.biz_code,
        fp16=args.fp16,
    )


def run_motion_mode(args: argparse.Namespace) -> int:
    repo_root = Path(__file__).resolve().parents[1]
    param_tag, default_yaml_path, policy_onnx_path, policy_mnn_path = (
        resolve_motion_paths(args.robot, args.motion, repo_root)
    )

    existing_config = load_yaml_mapping(default_yaml_path)
    metadata_config = load_tracking_policy_metadata(policy_onnx_path)
    next_config = build_tracking_yaml(existing_config, param_tag, metadata_config)
    next_yaml = dump_yaml_mapping(next_config)

    temp_mnn_path = policy_mnn_path.with_name(f"{policy_mnn_path.name}.tmp")
    if temp_mnn_path.exists():
        temp_mnn_path.unlink()

    return_code = convert_onnx_to_mnn(
        input_path=policy_onnx_path,
        output_path=temp_mnn_path,
        converter=args.converter,
        biz_code=args.biz_code,
        fp16=args.fp16,
    )
    if return_code != 0:
        if temp_mnn_path.exists():
            temp_mnn_path.unlink()
        return return_code
    if not temp_mnn_path.exists():
        print(f"Converter did not create expected output: {temp_mnn_path}", file=sys.stderr)
        return 1

    os.replace(temp_mnn_path, policy_mnn_path)
    atomic_write_text(default_yaml_path, next_yaml)
    print(f"Updated policy: {policy_mnn_path}")
    print(f"Updated config: {default_yaml_path}")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert policy.onnx, or another ONNX model, to .mnn format."
    )
    parser.add_argument(
        "motion",
        nargs="?",
        help=(
            "Optional motion name from task_motion/default.yaml. When set, the "
            "script resolves the rl_tracking_motion_runner param_tag, reads "
            "policy metadata from that motion's policy.onnx, updates default.yaml, "
            "and writes policy.mnn."
        ),
    )
    parser.add_argument(
        "--robot",
        default="pm01_edu",
        help="Robot config name used in motion mode. Default: pm01_edu",
    )
    parser.add_argument(
        "-i",
        "--input",
        help="Input ONNX model path for direct conversion mode. Default: policy.onnx",
    )
    parser.add_argument(
        "-o",
        "--output",
        help="Output MNN model path for direct conversion mode. Default: policy.mnn",
    )
    parser.add_argument(
        "--converter",
        help=(
            "Optional path to MNNConvert. If omitted, the script first imports "
            "PyMNN's converter, then checks $MNNCONVERT, PATH, and common local "
            "build paths."
        ),
    )
    parser.add_argument(
        "--biz-code",
        default="MNN",
        help="bizCode value passed to MNNConvert. Default: MNN",
    )
    parser.add_argument(
        "--fp16",
        action="store_true",
        help="Ask MNNConvert to store weights in FP16 when supported.",
    )
    args = parser.parse_args()
    if args.motion and (args.input is not None or args.output is not None):
        parser.error("motion mode resolves policy paths automatically; do not pass --input or --output")
    return args


def main() -> int:
    args = parse_args()

    try:
        if args.motion:
            return run_motion_mode(args)
        return run_direct_mode(args)
    except PolicySyncError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
