#include "mujoco_log_replay/mujoco_log_replay.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <unordered_set>

#include <mcap/reader.hpp>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

namespace mujoco_log_replay {
namespace {

constexpr int kSchemaVersion = 2;
constexpr std::string_view kJointCommandFeedbackInvocation =
    "joint_command_feedback";
constexpr std::string_view kRobotStateInvocation = "robot_state";
constexpr std::string_view kMetadataName =
    "rl_tracking_motion_policy_io_metadata";
constexpr double kTorqueWarningEpsilon = 1e-9;

std::runtime_error Error(const std::string& message) {
  return std::runtime_error("[mujoco_log_replay] " + message);
}

bool Exists(const std::filesystem::path& path) {
  std::error_code error;
  return std::filesystem::exists(path, error);
}

std::filesystem::path NormalizeExistingPath(
    const std::filesystem::path& path) {
  std::error_code error;
  auto canonical = std::filesystem::weakly_canonical(path, error);
  if (!error) {
    return canonical;
  }
  return path.lexically_normal();
}

int64_t JsonIntValue(const nlohmann::json& json, std::string_view key) {
  const std::string key_string(key);
  if (!json.contains(key_string)) {
    throw Error("missing JSON field: " + key_string);
  }
  return json.at(key_string).get<int64_t>();
}

uint64_t JsonUint64Value(const nlohmann::json& json, std::string_view key) {
  const int64_t value = JsonIntValue(json, key);
  if (value < 0) {
    throw Error("negative timestamp in JSON field: " + std::string(key));
  }
  return static_cast<uint64_t>(value);
}

double JsonDoubleValue(const nlohmann::json& json, std::string_view key) {
  const std::string key_string(key);
  if (!json.contains(key_string)) {
    throw Error("missing JSON field: " + key_string);
  }
  return json.at(key_string).get<double>();
}

std::vector<double> JsonDoubleArrayValue(
    const nlohmann::json& json, std::string_view key,
    std::string_view object_name = "JSON object") {
  const std::string key_string(key);
  if (!json.contains(key_string)) {
    throw Error("missing " + std::string(object_name) +
                " field: " + key_string);
  }
  const auto& array = json.at(key_string);
  if (!array.is_array()) {
    throw Error(std::string(object_name) + " field is not an array: " +
                key_string);
  }

  std::vector<double> values;
  values.reserve(array.size());
  for (const auto& value : array) {
    values.push_back(value.get<double>());
  }
  return values;
}

std::vector<std::string> JsonStringArrayValue(const std::string& value) {
  nlohmann::json array = nlohmann::json::parse(value);
  if (!array.is_array()) {
    throw Error("metadata value is not a string array");
  }

  std::vector<std::string> values;
  values.reserve(array.size());
  for (const auto& item : array) {
    values.push_back(item.get<std::string>());
  }
  return values;
}

void SetMetadataFromMap(const mcap::KeyValueMap& values,
                        ReplayMetadata* metadata) {
  auto it = values.find("schema_version");
  if (it != values.end() && !it->second.empty()) {
    metadata->schema_version = std::stoi(it->second);
  }

  it = values.find("runner");
  if (it != values.end()) {
    metadata->runner = it->second;
  }

  it = values.find("param_tag");
  if (it != values.end()) {
    metadata->param_tag = it->second;
  }

  it = values.find("policy_file");
  if (it != values.end()) {
    metadata->policy_file = it->second;
  }

  it = values.find("joint_command_feedback_topic");
  if (it != values.end()) {
    metadata->joint_command_feedback_topic = it->second;
  }

  it = values.find("robot_state_topic");
  if (it != values.end()) {
    metadata->robot_state_topic = it->second;
  }

  it = values.find("command_joint_names");
  if (it != values.end()) {
    metadata->command_joint_names = JsonStringArrayValue(it->second);
  }

  it = values.find("state_joint_names");
  if (it != values.end()) {
    metadata->state_joint_names = JsonStringArrayValue(it->second);
  }
}

ReplayMetadata ReadMetadata(const std::filesystem::path& log_path) {
  std::ifstream stream(log_path, std::ios::binary);
  if (!stream) {
    throw Error("failed to open MCAP for metadata scan: " + log_path.string());
  }

  mcap::FileStreamReader file_reader(stream);
  const uint64_t size = file_reader.size();
  if (size <= sizeof(mcap::Magic) * 2) {
    throw Error("MCAP is too small: " + log_path.string());
  }

  ReplayMetadata metadata;
  mcap::RecordReader record_reader(file_reader, sizeof(mcap::Magic),
                                   size - sizeof(mcap::Magic));
  while (const auto record = record_reader.next()) {
    if (record->opcode != mcap::OpCode::Metadata) {
      continue;
    }

    mcap::Metadata record_metadata;
    auto status = mcap::McapReader::ParseMetadata(*record, &record_metadata);
    if (!status.ok()) {
      throw Error("failed to parse MCAP metadata: " + status.message);
    }

    if (record_metadata.name == kMetadataName ||
        record_metadata.metadata.contains("command_joint_names")) {
      SetMetadataFromMap(record_metadata.metadata, &metadata);
    }
  }

  if (!record_reader.status().ok()) {
    throw Error("failed during MCAP metadata scan: " +
                record_reader.status().message);
  }

  return metadata;
}

bool IsJointCommandMessage(const nlohmann::json& payload,
                           const std::string& channel_topic,
                           const ReplayMetadata& metadata) {
  const std::string invocation = payload.value("invocation", "");
  if (invocation == kJointCommandFeedbackInvocation) {
    return true;
  }

  const std::string payload_topic = payload.value("topic", "");
  return channel_topic == metadata.joint_command_feedback_topic ||
         payload_topic == metadata.joint_command_feedback_topic;
}

bool IsRobotStateMessage(const nlohmann::json& payload,
                         const std::string& channel_topic,
                         const ReplayMetadata& metadata) {
  const std::string invocation = payload.value("invocation", "");
  if (invocation == kRobotStateInvocation) {
    return true;
  }

  const std::string payload_topic = payload.value("topic", "");
  return channel_topic == metadata.robot_state_topic ||
         payload_topic == metadata.robot_state_topic;
}

void ValidatePayloadSchemaVersion(const nlohmann::json& payload,
                                  std::string_view invocation) {
  if (!payload.contains("schema_version")) {
    throw Error("missing schema_version in " + std::string(invocation) +
                " payload");
  }
  const int schema_version = payload.at("schema_version").get<int>();
  if (schema_version != kSchemaVersion) {
    std::ostringstream error;
    error << "unsupported " << invocation << " payload schema_version "
          << schema_version << ", expected " << kSchemaVersion;
    throw Error(error.str());
  }
}

ReplayCommand ParseCommandPayload(const nlohmann::json& payload,
                                  uint64_t message_log_time_ns,
                                  const std::string& channel_topic) {
  ValidatePayloadSchemaVersion(payload, kJointCommandFeedbackInvocation);
  ReplayCommand command;
  command.iter = JsonIntValue(payload, "iter");
  command.runner_time_step = JsonIntValue(payload, "runner_time_step");
  command.runner_period_s = JsonDoubleValue(payload, "runner_period_s");
  command.log_time_ns = payload.contains("log_time_ns")
                            ? JsonUint64Value(payload, "log_time_ns")
                            : message_log_time_ns;
  command.topic = payload.value("topic", channel_topic);

  if (!payload.contains("joint_command_feedback") ||
      !payload.at("joint_command_feedback").is_object()) {
    throw Error("missing joint_command_feedback object");
  }
  const auto& joint_command = payload.at("joint_command_feedback");
  command.position = JsonDoubleArrayValue(joint_command, "position");
  command.velocity = JsonDoubleArrayValue(joint_command, "velocity");
  command.feed_forward_torque =
      JsonDoubleArrayValue(joint_command, "feed_forward_torque");
  command.torque = JsonDoubleArrayValue(joint_command, "torque");
  command.stiffness = JsonDoubleArrayValue(joint_command, "stiffness");
  command.damping = JsonDoubleArrayValue(joint_command, "damping");
  return command;
}

ReplayJointState ParseJointState(const nlohmann::json& json,
                                 std::string_view object_name) {
  ReplayJointState state;
  state.position = JsonDoubleArrayValue(json, "position", object_name);
  state.velocity = JsonDoubleArrayValue(json, "velocity", object_name);
  state.torque = JsonDoubleArrayValue(json, "torque", object_name);
  return state;
}

ReplayImuState ParseImuState(const nlohmann::json& json) {
  ReplayImuState state;
  state.quaternion_wxyz =
      JsonDoubleArrayValue(json, "quaternion_wxyz", "imu");
  state.rpy = JsonDoubleArrayValue(json, "rpy", "imu");
  state.linear_acceleration =
      JsonDoubleArrayValue(json, "linear_acceleration", "imu");
  state.angular_velocity =
      JsonDoubleArrayValue(json, "angular_velocity", "imu");
  return state;
}

ReplayLinkState ParseLinkState(const nlohmann::json& json,
                               std::string_view object_name) {
  ReplayLinkState state;
  state.position = JsonDoubleArrayValue(json, "position", object_name);
  state.quaternion_wxyz =
      JsonDoubleArrayValue(json, "quaternion_wxyz", object_name);
  state.linear_velocity =
      JsonDoubleArrayValue(json, "linear_velocity", object_name);
  state.angular_velocity =
      JsonDoubleArrayValue(json, "angular_velocity", object_name);
  state.linear_acceleration =
      JsonDoubleArrayValue(json, "linear_acceleration", object_name);
  state.angular_acceleration =
      JsonDoubleArrayValue(json, "angular_acceleration", object_name);
  return state;
}

ReplayRobotState ParseRobotStatePayload(const nlohmann::json& payload,
                                        uint64_t message_log_time_ns,
                                        const std::string& channel_topic) {
  ValidatePayloadSchemaVersion(payload, kRobotStateInvocation);
  ReplayRobotState state;
  state.iter = JsonIntValue(payload, "iter");
  state.runner_time_step = JsonIntValue(payload, "runner_time_step");
  state.runner_period_s = JsonDoubleValue(payload, "runner_period_s");
  state.log_time_ns = payload.contains("log_time_ns")
                          ? JsonUint64Value(payload, "log_time_ns")
                          : message_log_time_ns;
  state.topic = payload.value("topic", channel_topic);

  if (!payload.contains("robot_state") ||
      !payload.at("robot_state").is_object()) {
    throw Error("missing robot_state object");
  }
  const auto& robot_state = payload.at("robot_state");
  state.joint_state =
      ParseJointState(robot_state.at("joint_state"), "joint_state");
  state.motor_state =
      ParseJointState(robot_state.at("motor_state"), "motor_state");
  state.imu = ParseImuState(robot_state.at("imu"));
  state.base_state_in_world = ParseLinkState(
      robot_state.at("base_state_in_world"), "base_state_in_world");
  state.simulated_base_state_in_world =
      ParseLinkState(robot_state.at("simulated_base_state_in_world"),
                     "simulated_base_state_in_world");
  state.selected_anchor_state = ParseLinkState(
      robot_state.at("selected_anchor_state"), "selected_anchor_state");
  state.selected_anchor_source =
      robot_state.at("selected_anchor_source").get<std::string>();
  return state;
}

bool IterInRange(int64_t iter, int64_t start_iter, int64_t end_iter) {
  if (iter < start_iter) {
    return false;
  }
  return end_iter < 0 || iter <= end_iter;
}

void ValidateCommandFieldSizes(const ReplayCommand& command) {
  const size_t num_joints = command.position.size();
  const auto check = [&](std::string_view field, size_t size) {
    if (size != num_joints) {
      std::ostringstream error;
      error << "iter " << command.iter << " has mismatched " << field
            << " length " << size << ", expected " << num_joints;
      throw Error(error.str());
    }
  };

  check("velocity", command.velocity.size());
  check("feed_forward_torque", command.feed_forward_torque.size());
  check("torque", command.torque.size());
  check("stiffness", command.stiffness.size());
  check("damping", command.damping.size());
}

bool HasNonzeroTorque(const ReplayCommand& command) {
  return std::any_of(command.torque.begin(), command.torque.end(),
                     [](double value) {
                       return std::abs(value) > kTorqueWarningEpsilon;
                     });
}

void AddWarning(ReplayLog* log, const std::string& warning) {
  log->warnings.push_back(warning);
}

void ValidateMetadata(const ReplayMetadata& metadata) {
  if (!metadata.schema_version.has_value()) {
    throw Error("MCAP metadata missing schema_version");
  }
  if (*metadata.schema_version != kSchemaVersion) {
    std::ostringstream error;
    error << "unsupported MCAP schema_version " << *metadata.schema_version
          << ", expected " << kSchemaVersion;
    throw Error(error.str());
  }
  if (metadata.joint_command_feedback_topic.empty()) {
    throw Error("MCAP metadata missing joint_command_feedback_topic");
  }
  if (metadata.robot_state_topic.empty()) {
    throw Error("MCAP metadata missing robot_state_topic");
  }
  if (metadata.command_joint_names.empty()) {
    throw Error("MCAP metadata missing command_joint_names");
  }
  if (metadata.state_joint_names.empty()) {
    throw Error("MCAP metadata missing state_joint_names");
  }
}

void ValidateRobotJointStateFieldSizes(const ReplayJointState& state,
                                       std::string_view object_name,
                                       int64_t iter) {
  const size_t num_joints = state.position.size();
  const auto check = [&](std::string_view field, size_t size) {
    if (size != num_joints) {
      std::ostringstream error;
      error << "iter " << iter << " has mismatched " << object_name << "."
            << field << " length " << size << ", expected " << num_joints;
      throw Error(error.str());
    }
  };

  check("velocity", state.velocity.size());
  check("torque", state.torque.size());
}

void ValidateSize(std::string_view name, size_t actual, size_t expected,
                  int64_t iter) {
  if (actual == expected) {
    return;
  }
  std::ostringstream error;
  error << "iter " << iter << " has " << name << " length " << actual
        << ", expected " << expected;
  throw Error(error.str());
}

void ValidateImuFieldSizes(const ReplayImuState& state, int64_t iter) {
  ValidateSize("imu.quaternion_wxyz", state.quaternion_wxyz.size(), 4, iter);
  ValidateSize("imu.rpy", state.rpy.size(), 3, iter);
  ValidateSize("imu.linear_acceleration", state.linear_acceleration.size(), 3,
               iter);
  ValidateSize("imu.angular_velocity", state.angular_velocity.size(), 3, iter);
}

void ValidateLinkFieldSizes(const ReplayLinkState& state,
                            std::string_view object_name, int64_t iter) {
  ValidateSize(std::string(object_name) + ".position", state.position.size(),
               3, iter);
  ValidateSize(std::string(object_name) + ".quaternion_wxyz",
               state.quaternion_wxyz.size(), 4, iter);
  ValidateSize(std::string(object_name) + ".linear_velocity",
               state.linear_velocity.size(), 3, iter);
  ValidateSize(std::string(object_name) + ".angular_velocity",
               state.angular_velocity.size(), 3, iter);
  ValidateSize(std::string(object_name) + ".linear_acceleration",
               state.linear_acceleration.size(), 3, iter);
  ValidateSize(std::string(object_name) + ".angular_acceleration",
               state.angular_acceleration.size(), 3, iter);
}

std::array<double, 4> NormalizedQuaternionWxyz(
    const std::vector<double>& quaternion) {
  if (quaternion.size() != 4) {
    return {1.0, 0.0, 0.0, 0.0};
  }
  const double norm =
      std::sqrt(quaternion[0] * quaternion[0] +
                quaternion[1] * quaternion[1] +
                quaternion[2] * quaternion[2] +
                quaternion[3] * quaternion[3]);
  if (norm <= 0.0) {
    return {1.0, 0.0, 0.0, 0.0};
  }
  return {quaternion[0] / norm, quaternion[1] / norm,
          quaternion[2] / norm, quaternion[3] / norm};
}

void CopyVectorToArray(const std::vector<double>& values, double* output,
                       size_t expected_size) {
  std::fill(output, output + expected_size, 0.0);
  std::copy_n(values.begin(), std::min(values.size(), expected_size), output);
}

void ValidateRobotStateFieldSizes(const ReplayRobotState& state) {
  ValidateRobotJointStateFieldSizes(state.joint_state, "joint_state",
                                    state.iter);
  ValidateRobotJointStateFieldSizes(state.motor_state, "motor_state",
                                    state.iter);
  if (state.motor_state.JointCount() != state.joint_state.JointCount()) {
    std::ostringstream error;
    error << "iter " << state.iter << " motor_state has "
          << state.motor_state.JointCount() << " joints, but joint_state has "
          << state.joint_state.JointCount();
    throw Error(error.str());
  }
  ValidateImuFieldSizes(state.imu, state.iter);
  ValidateLinkFieldSizes(state.base_state_in_world, "base_state_in_world",
                         state.iter);
  ValidateLinkFieldSizes(state.simulated_base_state_in_world,
                         "simulated_base_state_in_world", state.iter);
  ValidateLinkFieldSizes(state.selected_anchor_state,
                         "selected_anchor_state", state.iter);
}

void ValidateAndAnnotateReplayLog(
    ReplayLog* log, const std::optional<RobotModelInfo>& robot_model) {
  ValidateMetadata(log->metadata);

  if (log->commands.empty()) {
    throw Error("no joint_command_feedback messages found in selected range");
  }
  if (log->robot_states.empty()) {
    throw Error("no robot_state messages found in selected range");
  }

  const size_t metadata_joint_count =
      log->metadata.command_joint_names.size();
  const size_t metadata_state_joint_count =
      log->metadata.state_joint_names.size();
  if (metadata_state_joint_count != metadata_joint_count) {
    std::ostringstream error;
    error << "MCAP state_joint_names has " << metadata_state_joint_count
          << " joints, but command_joint_names has " << metadata_joint_count;
    throw Error(error.str());
  }
  const std::optional<int> robot_joint_count =
      robot_model.has_value()
          ? std::optional<int>(robot_model->num_total_joints)
          : std::nullopt;
  if (robot_joint_count.has_value() &&
      metadata_joint_count != static_cast<size_t>(*robot_joint_count)) {
    std::ostringstream error;
    error << "MCAP command_joint_names has " << metadata_joint_count
          << " joints, but robot model has " << *robot_joint_count;
    throw Error(error.str());
  }

  const size_t command_joint_count = log->commands.front().JointCount();
  bool torque_warning_added = false;
  for (const auto& command : log->commands) {
    ValidateCommandFieldSizes(command);
    if (command.JointCount() != metadata_joint_count) {
      std::ostringstream error;
      error << "iter " << command.iter << " command has "
            << command.JointCount() << " joints, but metadata has "
            << metadata_joint_count;
      throw Error(error.str());
    }
    if (HasNonzeroTorque(command)) {
      log->has_nonzero_torque = true;
      if (!torque_warning_added) {
        AddWarning(log,
                   "nonzero torque field detected; SimCommand has no torque "
                   "slot, so replay ignores torque and publishes feed-forward "
                   "torque only");
        torque_warning_added = true;
      }
    }
  }

  for (const auto& state : log->robot_states) {
    ValidateRobotStateFieldSizes(state);
    if (state.JointCount() != command_joint_count) {
      std::ostringstream error;
      error << "iter " << state.iter << " robot_state has "
            << state.JointCount() << " joints, but command has "
            << command_joint_count;
      throw Error(error.str());
    }
    if (state.JointCount() != metadata_joint_count) {
      std::ostringstream error;
      error << "iter " << state.iter << " robot_state has "
            << state.JointCount() << " joints, but metadata has "
            << metadata_joint_count;
      throw Error(error.str());
    }
  }

  std::unordered_set<int64_t> seen_iters;
  bool duplicate_iter_warning_added = false;
  bool time_jump_warning_added = false;
  for (size_t i = 0; i < log->commands.size(); ++i) {
    const auto& command = log->commands[i];
    if (!seen_iters.insert(command.iter).second &&
        !duplicate_iter_warning_added) {
      AddWarning(log, "duplicate iter values detected in replay commands");
      duplicate_iter_warning_added = true;
    }

    if (i == 0) {
      continue;
    }

    const auto& previous = log->commands[i - 1];
    const int64_t delta_ns =
        static_cast<int64_t>(command.log_time_ns) -
        static_cast<int64_t>(previous.log_time_ns);
    const double expected_period_s =
        previous.runner_period_s > 0.0 ? previous.runner_period_s
                                       : command.runner_period_s;
    const double delta_s = static_cast<double>(delta_ns) * 1e-9;
    if (delta_ns < 0 && !time_jump_warning_added) {
      AddWarning(log,
                 "log_time_ns is not monotonic after iter/log_time sorting");
      time_jump_warning_added = true;
    } else if (expected_period_s > 0.0 &&
               delta_s > std::max(1.0, expected_period_s * 10.0) &&
               !time_jump_warning_added) {
      AddWarning(log, "large log_time_ns gap detected in replay commands");
      time_jump_warning_added = true;
    }
  }
}

std::optional<int> SumJointsInModelConfig(
    const std::filesystem::path& model_config_path) {
  YAML::Node root = YAML::LoadFile(model_config_path.string());
  YAML::Node limbs = root["limbs"];
  if (!limbs || !limbs.IsSequence()) {
    throw Error("robot model config missing limbs sequence: " +
                model_config_path.string());
  }

  int total = 0;
  for (const auto& limb : limbs) {
    YAML::Node joints = limb["joints"];
    if (!joints || !joints.IsSequence()) {
      throw Error("robot model config limb missing joints sequence: " +
                  model_config_path.string());
    }
    total += static_cast<int>(joints.size());
  }
  return total;
}

std::string FallbackLcmUrl() { return "udpm://239.255.76.67:7667?ttl=0"; }

}  // namespace

double ReplayLog::StartTimeS() const {
  if (commands.empty()) {
    return 0.0;
  }
  return static_cast<double>(commands.front().log_time_ns) * 1e-9;
}

double ReplayLog::EndTimeS() const {
  if (commands.empty()) {
    return 0.0;
  }
  return static_cast<double>(commands.back().log_time_ns) * 1e-9;
}

double ReplayLog::RobotStateStartTimeS() const {
  if (robot_states.empty()) {
    return 0.0;
  }
  return static_cast<double>(robot_states.front().log_time_ns) * 1e-9;
}

double ReplayLog::RobotStateEndTimeS() const {
  if (robot_states.empty()) {
    return 0.0;
  }
  return static_cast<double>(robot_states.back().log_time_ns) * 1e-9;
}

std::filesystem::path ResolveRepoRoot() {
  const char* env_root = std::getenv("ENGINEAI_ROBOTICS_DIR");
  if (env_root != nullptr && env_root[0] != '\0') {
    return NormalizeExistingPath(env_root);
  }

  std::filesystem::path current = std::filesystem::current_path();
  while (!current.empty()) {
    if (Exists(current / "assets" / "config") &&
        Exists(current / "simulation" / "mujoco")) {
      return NormalizeExistingPath(current);
    }
    current = current.parent_path();
  }

  return NormalizeExistingPath(std::filesystem::current_path());
}

std::filesystem::path ResolveLogPath(const std::filesystem::path& log_path,
                                     const std::filesystem::path& repo_root) {
  if (log_path.is_absolute()) {
    return NormalizeExistingPath(log_path);
  }

  const auto cwd_candidate = std::filesystem::current_path() / log_path;
  if (Exists(cwd_candidate)) {
    return NormalizeExistingPath(cwd_candidate);
  }

  return NormalizeExistingPath(repo_root / log_path);
}

std::optional<RobotModelInfo> LoadRobotModelInfo(
    const std::filesystem::path& repo_root, const std::string& robot,
    std::vector<std::string>* warnings) {
  const auto model_config_path =
      repo_root / "assets" / "config" / robot / "model" / "default.yaml";
  if (!Exists(model_config_path)) {
    if (warnings != nullptr) {
      warnings->push_back("robot model config not found, skipped robot joint "
                          "count validation: " +
                          model_config_path.string());
    }
    return std::nullopt;
  }

  RobotModelInfo info;
  info.model_config_path = NormalizeExistingPath(model_config_path);
  info.num_total_joints = *SumJointsInModelConfig(info.model_config_path);
  return info;
}

std::string DefaultLcmUrlForRobot(const std::filesystem::path& repo_root,
                                  const std::string& robot,
                                  std::vector<std::string>* warnings) {
  const auto lcm_config_path =
      repo_root / "assets" / "config" / robot / "lcm" / "default.yaml";
  if (!Exists(lcm_config_path)) {
    if (warnings != nullptr) {
      warnings->push_back("LCM config not found, using fallback URL: " +
                          lcm_config_path.string());
    }
    return FallbackLcmUrl();
  }

  try {
    YAML::Node root = YAML::LoadFile(lcm_config_path.string());
    const bool multicast = root["multicast"].as<bool>();
    const std::string ip_port = root["ip_port"].as<std::string>();
    const int ttl = multicast ? root["ttl"].as<int>() : 0;
    return "udpm://" + ip_port + "?ttl=" + std::to_string(ttl);
  } catch (const std::exception& error) {
    if (warnings != nullptr) {
      warnings->push_back("failed to parse LCM config, using fallback URL: " +
                          std::string(error.what()));
    }
    return FallbackLcmUrl();
  }
}

ReplayMode ParseReplayMode(std::string_view value) {
  if (value == "command") {
    return ReplayMode::kCommand;
  }
  if (value == "state") {
    return ReplayMode::kState;
  }
  throw Error("unsupported replay mode: " + std::string(value));
}

std::string ReplayModeName(ReplayMode mode) {
  switch (mode) {
    case ReplayMode::kCommand:
      return "command";
    case ReplayMode::kState:
      return "state";
  }
  return "unknown";
}

std::string DefaultTopicForMode(ReplayMode mode) {
  switch (mode) {
    case ReplayMode::kCommand:
      return "sim_command";
    case ReplayMode::kState:
      return "sim_replay_state";
  }
  return "sim_command";
}

ReplayLog LoadReplayLog(const LoadOptions& options) {
  if (options.log_path.empty()) {
    throw Error("--log is required");
  }
  if (!Exists(options.log_path)) {
    throw Error("log file not found: " + options.log_path.string());
  }
  if (options.end_iter >= 0 && options.start_iter > options.end_iter) {
    throw Error("--start-iter must be <= --end-iter");
  }

  ReplayLog log;
  log.log_path = NormalizeExistingPath(options.log_path);
  log.metadata = ReadMetadata(log.log_path);
  ValidateMetadata(log.metadata);

  std::vector<std::string> robot_warnings;
  std::optional<RobotModelInfo> robot_model;
  if (!options.robot.empty()) {
    robot_model =
        LoadRobotModelInfo(options.repo_root, options.robot, &robot_warnings);
  }

  mcap::McapReader reader;
  auto status = reader.open(log.log_path.string());
  if (!status.ok()) {
    throw Error("failed to open MCAP: " + status.message);
  }

  status = reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);
  if (!status.ok()) {
    throw Error("failed to read MCAP summary: " + status.message);
  }

  for (const auto& message_view : reader.readMessages()) {
    const auto& message = message_view.message;
    const auto* data = reinterpret_cast<const char*>(message.data);
    const nlohmann::json payload =
        nlohmann::json::parse(std::string(data, data + message.dataSize));
    const std::string channel_topic =
        message_view.channel ? message_view.channel->topic : "";

    if (IsJointCommandMessage(payload, channel_topic, log.metadata)) {
      ReplayCommand command =
          ParseCommandPayload(payload, message.logTime, channel_topic);
      if (IterInRange(command.iter, options.start_iter, options.end_iter)) {
        log.commands.push_back(std::move(command));
      }
    } else if (IsRobotStateMessage(payload, channel_topic, log.metadata)) {
      ReplayRobotState state =
          ParseRobotStatePayload(payload, message.logTime, channel_topic);
      if (IterInRange(state.iter, options.start_iter, options.end_iter)) {
        log.robot_states.push_back(std::move(state));
      }
    }
  }
  reader.close();

  std::sort(log.commands.begin(), log.commands.end(),
            [](const ReplayCommand& left, const ReplayCommand& right) {
              if (left.iter != right.iter) {
                return left.iter < right.iter;
              }
              return left.log_time_ns < right.log_time_ns;
            });
  std::sort(log.robot_states.begin(), log.robot_states.end(),
            [](const ReplayRobotState& left, const ReplayRobotState& right) {
              if (left.log_time_ns != right.log_time_ns) {
                return left.log_time_ns < right.log_time_ns;
              }
              return left.iter < right.iter;
            });

  ValidateAndAnnotateReplayLog(&log, robot_model);
  log.warnings.insert(log.warnings.end(), robot_warnings.begin(),
                      robot_warnings.end());
  return log;
}

data::SimCommand ToSimCommand(const ReplayCommand& command) {
  data::SimCommand message;
  message.timestamp = static_cast<double>(command.log_time_ns) * 1e-9;
  message.num_ranges = static_cast<int32_t>(command.JointCount());
  message.joint_position = command.position;
  message.joint_velocity = command.velocity;
  message.joint_feed_forward_torque = command.feed_forward_torque;
  message.joint_stiffness = command.stiffness;
  message.joint_damping = command.damping;
  return message;
}

data::SimState ToSimReplayState(const ReplayRobotState& state) {
  data::SimState message{};
  message.timestamp = static_cast<double>(state.log_time_ns) * 1e-9;
  message.num_ranges = static_cast<int32_t>(state.JointCount());
  message.joint_position = state.joint_state.position;
  message.joint_velocity = state.joint_state.velocity;
  message.joint_torque = state.joint_state.torque;

  CopyVectorToArray(state.selected_anchor_state.position,
                    message.base_link_position, 3);
  CopyVectorToArray(state.selected_anchor_state.linear_velocity,
                    message.base_link_linear_velocity, 3);
  const auto base_quaternion =
      NormalizedQuaternionWxyz(state.selected_anchor_state.quaternion_wxyz);
  std::copy(base_quaternion.begin(), base_quaternion.end(),
            message.base_link_quaternion);
  CopyVectorToArray(state.selected_anchor_state.angular_velocity,
                    message.base_link_angular_velocity, 3);

  message.imu_link_quaternion[0] = 1.0;
  const auto imu_quaternion = NormalizedQuaternionWxyz(state.imu.quaternion_wxyz);
  std::copy(imu_quaternion.begin(), imu_quaternion.end(),
            message.imu_sensor_quaternion);
  CopyVectorToArray(state.imu.linear_acceleration,
                    message.imu_sensor_linear_acceleration, 3);
  CopyVectorToArray(state.imu.angular_velocity,
                    message.imu_sensor_angular_velocity, 3);

  message.num_contact_ranges = 0;
  message.contact_force.clear();
  return message;
}

void PrintDryRunSummary(const ReplayLog& log,
                        const std::optional<RobotModelInfo>& robot_model,
                        std::ostream& out) {
  const size_t joint_count =
      log.commands.empty() ? 0U : log.commands.front().JointCount();
  const double duration_s =
      log.commands.empty() ? 0.0 : log.EndTimeS() - log.StartTimeS();
  const double runner_period_s =
      log.commands.empty() ? 0.0 : log.commands.front().runner_period_s;
  const size_t robot_state_joint_count =
      log.robot_states.empty() ? 0U : log.robot_states.front().JointCount();
  const double robot_state_duration_s =
      log.robot_states.empty()
          ? 0.0
          : log.RobotStateEndTimeS() - log.RobotStateStartTimeS();
  std::set<std::string> anchor_sources;
  for (const auto& state : log.robot_states) {
    anchor_sources.insert(state.selected_anchor_source);
  }

  out << "mujoco_log_replay dry-run summary\n";
  out << "  log: " << log.log_path << "\n";
  out << "  schema_version: ";
  if (log.metadata.schema_version.has_value()) {
    out << *log.metadata.schema_version;
  } else {
    out << "unknown";
  }
  out << "\n";
  out << "  runner: " << log.metadata.runner << "\n";
  out << "  param_tag: " << log.metadata.param_tag << "\n";
  out << "  policy_file: " << log.metadata.policy_file << "\n";
  out << "  command_count: " << log.commands.size() << "\n";
  out << "  joint_count: " << joint_count << "\n";
  out << "  robot_state_count: " << log.robot_states.size() << "\n";
  out << "  robot_state_joint_count: " << robot_state_joint_count << "\n";
  out << "  metadata_command_joint_count: "
      << log.metadata.command_joint_names.size() << "\n";
  if (robot_model.has_value()) {
    out << "  robot_model_joint_count: " << robot_model->num_total_joints
        << "\n";
    out << "  robot_model_config: " << robot_model->model_config_path << "\n";
  } else {
    out << "  robot_model_joint_count: unavailable\n";
  }
  out << "  time_range_s: " << std::fixed << std::setprecision(6)
      << log.StartTimeS() << " -> " << log.EndTimeS()
      << " (duration " << duration_s << ")\n";
  out << "  robot_state_time_range_s: " << std::fixed
      << std::setprecision(6) << log.RobotStateStartTimeS() << " -> "
      << log.RobotStateEndTimeS() << " (duration "
      << robot_state_duration_s << ")\n";
  out << "  runner_period_s: " << std::fixed << std::setprecision(6)
      << runner_period_s << "\n";
  out << "  selected_anchor_sources:";
  if (anchor_sources.empty()) {
    out << " none";
  } else {
    for (const auto& source : anchor_sources) {
      out << " " << source;
    }
  }
  out << "\n";
  out << "  nonzero_torque: " << (log.has_nonzero_torque ? "yes" : "no")
      << "\n";
  out << "  warnings: " << log.warnings.size() << "\n";
  for (const auto& warning : log.warnings) {
    out << "    - " << warning << "\n";
  }
}

}  // namespace mujoco_log_replay
