#ifndef SIMULATION_MUJOCO_SRC_REPLAY_INCLUDE_MUJOCO_LOG_REPLAY_MUJOCO_LOG_REPLAY_H_
#define SIMULATION_MUJOCO_SRC_REPLAY_INCLUDE_MUJOCO_LOG_REPLAY_MUJOCO_LOG_REPLAY_H_

#include <cstdint>
#include <filesystem>
#include <iosfwd>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "lcm_data/SimCommand.hpp"
#include "lcm_data/SimState.hpp"

namespace mujoco_log_replay {

enum class ReplayMode {
  kCommand,
  kState,
};

struct ReplayCommand {
  int64_t iter = 0;
  int64_t runner_time_step = 0;
  double runner_period_s = 0.0;
  uint64_t log_time_ns = 0;
  std::string topic;
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> feed_forward_torque;
  std::vector<double> torque;
  std::vector<double> stiffness;
  std::vector<double> damping;

  size_t JointCount() const { return position.size(); }
};

struct ReplayJointState {
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> torque;

  size_t JointCount() const { return position.size(); }
};

struct ReplayImuState {
  std::vector<double> quaternion_wxyz;
  std::vector<double> rpy;
  std::vector<double> linear_acceleration;
  std::vector<double> angular_velocity;
};

struct ReplayLinkState {
  std::vector<double> position;
  std::vector<double> quaternion_wxyz;
  std::vector<double> linear_velocity;
  std::vector<double> angular_velocity;
  std::vector<double> linear_acceleration;
  std::vector<double> angular_acceleration;
};

struct ReplayRobotState {
  int64_t iter = 0;
  int64_t runner_time_step = 0;
  double runner_period_s = 0.0;
  uint64_t log_time_ns = 0;
  std::string topic;
  ReplayJointState joint_state;
  ReplayJointState motor_state;
  ReplayImuState imu;
  ReplayLinkState base_state_in_world;
  ReplayLinkState simulated_base_state_in_world;
  ReplayLinkState selected_anchor_state;
  std::string selected_anchor_source;

  size_t JointCount() const { return joint_state.JointCount(); }
};

struct ReplayMetadata {
  std::optional<int> schema_version;
  std::string runner;
  std::string param_tag;
  std::string policy_file;
  std::string joint_command_feedback_topic;
  std::string robot_state_topic;
  std::vector<std::string> command_joint_names;
  std::vector<std::string> state_joint_names;
};

struct ReplayLog {
  std::filesystem::path log_path;
  ReplayMetadata metadata;
  std::vector<ReplayCommand> commands;
  std::vector<ReplayRobotState> robot_states;
  std::vector<std::string> warnings;
  bool has_nonzero_torque = false;

  double StartTimeS() const;
  double EndTimeS() const;
  double RobotStateStartTimeS() const;
  double RobotStateEndTimeS() const;
};

struct LoadOptions {
  std::filesystem::path log_path;
  std::filesystem::path repo_root;
  std::string robot;
  int64_t start_iter = 0;
  int64_t end_iter = -1;
};

struct RobotModelInfo {
  std::filesystem::path model_config_path;
  int num_total_joints = 0;
};

std::filesystem::path ResolveRepoRoot();
std::filesystem::path ResolveLogPath(const std::filesystem::path& log_path,
                                     const std::filesystem::path& repo_root);
std::optional<RobotModelInfo> LoadRobotModelInfo(
    const std::filesystem::path& repo_root, const std::string& robot,
    std::vector<std::string>* warnings);
std::string DefaultLcmUrlForRobot(const std::filesystem::path& repo_root,
                                  const std::string& robot,
                                  std::vector<std::string>* warnings);
ReplayMode ParseReplayMode(std::string_view value);
std::string ReplayModeName(ReplayMode mode);
std::string DefaultTopicForMode(ReplayMode mode);

ReplayLog LoadReplayLog(const LoadOptions& options);
data::SimCommand ToSimCommand(const ReplayCommand& command);
data::SimState ToSimReplayState(const ReplayRobotState& state);
void PrintDryRunSummary(const ReplayLog& log,
                        const std::optional<RobotModelInfo>& robot_model,
                        std::ostream& out);

}  // namespace mujoco_log_replay

#endif  // SIMULATION_MUJOCO_SRC_REPLAY_INCLUDE_MUJOCO_LOG_REPLAY_MUJOCO_LOG_REPLAY_H_
