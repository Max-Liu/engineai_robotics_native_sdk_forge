#include "mujoco_log_replay/mujoco_log_replay.h"

#include <chrono>
#include <filesystem>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <mcap/writer.hpp>
#include <nlohmann/json.hpp>

namespace mujoco_log_replay {
namespace {

std::filesystem::path MakeTempDir(std::string_view test_name) {
  const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  return std::filesystem::temp_directory_path() /
         ("mujoco_log_replay_" + std::string(test_name) + "_" +
          std::to_string(now));
}

nlohmann::json MakePayload(int iter, uint64_t log_time_ns,
                           const nlohmann::json& joint_command,
                           int schema_version = 2) {
  return {
      {"schema_version", schema_version},
      {"runner", "rl_tracking_motion_runner"},
      {"param_tag", "test_motion"},
      {"policy_file", "test_motion/policies/policy.mnn"},
      {"invocation", "joint_command_feedback"},
      {"iter", iter},
      {"runner_time_step", iter + 10},
      {"runner_period_s", 0.02},
      {"log_time_ns", log_time_ns},
      {"topic", "/hardware/joint_command_feedback"},
      {"joint_command_feedback", joint_command},
  };
}

nlohmann::json MakeJointState(const std::vector<double>& position,
                              const std::vector<double>& velocity,
                              const std::vector<double>& torque) {
  return {
      {"position", position},
      {"velocity", velocity},
      {"torque", torque},
  };
}

nlohmann::json MakeLinkState(double start) {
  return {
      {"position", {start, start + 1.0, start + 2.0}},
      {"quaternion_wxyz", {start + 3.0, start + 4.0, start + 5.0,
                           start + 6.0}},
      {"linear_velocity", {start + 7.0, start + 8.0, start + 9.0}},
      {"angular_velocity", {start + 10.0, start + 11.0, start + 12.0}},
      {"linear_acceleration", {start + 13.0, start + 14.0, start + 15.0}},
      {"angular_acceleration", {start + 16.0, start + 17.0, start + 18.0}},
  };
}

nlohmann::json MakeRobotStatePayload(
    int iter, uint64_t log_time_ns, const std::vector<double>& position,
    const std::string& selected_anchor_source = "base_state_in_world",
    int schema_version = 2) {
  const std::vector<double> velocity(position.size(), 0.1);
  const std::vector<double> torque(position.size(), 0.0);
  return {
      {"schema_version", schema_version},
      {"runner", "rl_tracking_motion_runner"},
      {"param_tag", "test_motion"},
      {"policy_file", "test_motion/policies/policy.mnn"},
      {"invocation", "robot_state"},
      {"iter", iter},
      {"runner_time_step", iter + 10},
      {"runner_period_s", 0.02},
      {"log_time_ns", log_time_ns},
      {"topic", "/rl_tracking_motion/robot_state"},
      {"robot_state",
       {
           {"joint_state", MakeJointState(position, velocity, torque)},
           {"motor_state", MakeJointState(position, velocity, torque)},
           {"imu",
            {
                {"quaternion_wxyz", {1.0, 0.0, 0.0, 0.0}},
                {"rpy", {0.0, 0.0, 0.0}},
                {"linear_acceleration", {0.0, 0.0, 9.8}},
                {"angular_velocity", {0.1, 0.2, 0.3}},
            }},
           {"base_state_in_world", MakeLinkState(10.0)},
           {"simulated_base_state_in_world", MakeLinkState(30.0)},
           {"selected_anchor_state", MakeLinkState(50.0)},
           {"selected_anchor_source", selected_anchor_source},
       }},
  };
}

nlohmann::json MakeJointCommand(const std::vector<double>& position,
                                const std::vector<double>& velocity,
                                const std::vector<double>& torque) {
  return {
      {"position", position},
      {"velocity", velocity},
      {"feed_forward_torque", std::vector<double>(position.size(), 0.0)},
      {"torque", torque},
      {"stiffness", std::vector<double>(position.size(), 10.0)},
      {"damping", std::vector<double>(position.size(), 1.0)},
  };
}

void WriteMcap(const std::filesystem::path& path,
               const std::vector<nlohmann::json>& command_payloads,
               const std::vector<nlohmann::json>& robot_state_payloads,
               const std::vector<std::string>& command_joint_names,
               int metadata_schema_version = 2) {
  mcap::McapWriter writer;
  mcap::McapWriterOptions options("json");
  options.noChunking = true;
  auto status = writer.open(path.string(), options);
  ASSERT_TRUE(status.ok()) << status.message;

  mcap::Schema schema("engineai.rl_tracking_motion.PolicyIo", "jsonschema",
                      "{}");
  writer.addSchema(schema);
  mcap::Channel channel("/hardware/joint_command_feedback", "json", schema.id,
                        {{"invocation", "joint_command_feedback"}});
  writer.addChannel(channel);
  mcap::Channel robot_state_channel("/rl_tracking_motion/robot_state", "json",
                                    schema.id,
                                    {{"invocation", "robot_state"}});
  writer.addChannel(robot_state_channel);

  mcap::Metadata metadata;
  metadata.name = "rl_tracking_motion_policy_io_metadata";
  metadata.metadata["schema_version"] = std::to_string(metadata_schema_version);
  metadata.metadata["runner"] = "rl_tracking_motion_runner";
  metadata.metadata["param_tag"] = "test_motion";
  metadata.metadata["policy_file"] = "test_motion/policies/policy.mnn";
  metadata.metadata["joint_command_feedback_topic"] =
      "/hardware/joint_command_feedback";
  metadata.metadata["robot_state_topic"] = "/rl_tracking_motion/robot_state";
  metadata.metadata["command_joint_names"] =
      nlohmann::json(command_joint_names).dump();
  metadata.metadata["state_joint_names"] =
      nlohmann::json(command_joint_names).dump();
  status = writer.write(metadata);
  ASSERT_TRUE(status.ok()) << status.message;

  uint32_t sequence = 0;
  for (const auto& payload : command_payloads) {
    const std::string payload_string = payload.dump();
    mcap::Message message;
    message.channelId = channel.id;
    message.sequence = sequence++;
    message.logTime = payload.at("log_time_ns").get<uint64_t>();
    message.publishTime = message.logTime;
    message.dataSize = payload_string.size();
    message.data =
        reinterpret_cast<const std::byte*>(payload_string.data());
    status = writer.write(message);
    ASSERT_TRUE(status.ok()) << status.message;
  }
  sequence = 0;
  for (const auto& payload : robot_state_payloads) {
    const std::string payload_string = payload.dump();
    mcap::Message message;
    message.channelId = robot_state_channel.id;
    message.sequence = sequence++;
    message.logTime = payload.at("log_time_ns").get<uint64_t>();
    message.publishTime = message.logTime;
    message.dataSize = payload_string.size();
    message.data =
        reinterpret_cast<const std::byte*>(payload_string.data());
    status = writer.write(message);
    ASSERT_TRUE(status.ok()) << status.message;
  }

  writer.close();
}

LoadOptions MakeLoadOptions(const std::filesystem::path& log_path) {
  LoadOptions options;
  options.log_path = log_path;
  options.repo_root = log_path.parent_path();
  return options;
}

TEST(MujocoLogReplayTest, ReadsSortsAndMapsCommands) {
  const auto dir = MakeTempDir("reads");
  std::filesystem::create_directories(dir);
  const auto log_path = dir / "commands.mcap";
  WriteMcap(log_path,
            {
                MakePayload(2, 120, MakeJointCommand({2.0, 3.0},
                                                     {0.2, 0.3}, {0.0, 0.0})),
                MakePayload(1, 100, MakeJointCommand({1.0, 2.0},
                                                     {0.1, 0.2}, {0.0, 0.0})),
            },
            {
                MakeRobotStatePayload(2, 121, {2.0, 3.0},
                                      "simulated_base_state_in_world"),
                MakeRobotStatePayload(1, 101, {1.0, 2.0}),
            },
            {"joint_a", "joint_b"});

  ReplayLog log = LoadReplayLog(MakeLoadOptions(log_path));

  ASSERT_EQ(log.commands.size(), 2U);
  ASSERT_EQ(log.robot_states.size(), 2U);
  EXPECT_EQ(log.commands[0].iter, 1);
  EXPECT_EQ(log.commands[1].iter, 2);
  EXPECT_EQ(log.robot_states[0].iter, 1);
  EXPECT_EQ(log.robot_states[0].JointCount(), 2U);
  EXPECT_EQ(log.robot_states[1].selected_anchor_source,
            "simulated_base_state_in_world");
  EXPECT_EQ(log.metadata.command_joint_names.size(), 2U);
  EXPECT_EQ(log.metadata.state_joint_names.size(), 2U);

  data::SimCommand message = ToSimCommand(log.commands[0]);
  EXPECT_EQ(message.num_ranges, 2);
  EXPECT_EQ(message.joint_position[0], 1.0);
  EXPECT_EQ(message.joint_velocity[1], 0.2);
  EXPECT_EQ(message.joint_feed_forward_torque.size(), 2U);
  EXPECT_EQ(message.joint_stiffness[0], 10.0);
  EXPECT_EQ(message.joint_damping[1], 1.0);

  std::filesystem::remove_all(dir);
}

TEST(MujocoLogReplayTest, CommandModeKeepsDefaultCommandTopic) {
  EXPECT_EQ(ParseReplayMode("command"), ReplayMode::kCommand);
  EXPECT_EQ(ParseReplayMode("state"), ReplayMode::kState);
  EXPECT_EQ(ReplayModeName(ReplayMode::kCommand), "command");
  EXPECT_EQ(DefaultTopicForMode(ReplayMode::kCommand), "sim_command");
  EXPECT_EQ(DefaultTopicForMode(ReplayMode::kState), "sim_replay_state");
  EXPECT_THROW(ParseReplayMode("unknown"), std::runtime_error);
}

TEST(MujocoLogReplayTest, MapsRobotStateToSimReplayState) {
  ReplayRobotState state;
  state.log_time_ns = 2500000000ULL;
  state.joint_state.position = {1.0, 2.0};
  state.joint_state.velocity = {0.1, 0.2};
  state.joint_state.torque = {3.0, 4.0};
  state.selected_anchor_state.position = {10.0, 11.0, 12.0};
  state.selected_anchor_state.quaternion_wxyz = {2.0, 0.0, 0.0, 0.0};
  state.selected_anchor_state.linear_velocity = {0.3, 0.4, 0.5};
  state.selected_anchor_state.angular_velocity = {0.6, 0.7, 0.8};
  state.imu.quaternion_wxyz = {0.5, 0.5, 0.5, 0.5};
  state.imu.linear_acceleration = {7.0, 8.0, 9.0};
  state.imu.angular_velocity = {1.1, 1.2, 1.3};

  data::SimState message = ToSimReplayState(state);

  EXPECT_DOUBLE_EQ(message.timestamp, 2.5);
  EXPECT_EQ(message.num_ranges, 2);
  EXPECT_EQ(message.joint_position, state.joint_state.position);
  EXPECT_EQ(message.joint_velocity, state.joint_state.velocity);
  EXPECT_EQ(message.joint_torque, state.joint_state.torque);
  EXPECT_DOUBLE_EQ(message.base_link_position[0], 10.0);
  EXPECT_DOUBLE_EQ(message.base_link_position[2], 12.0);
  EXPECT_DOUBLE_EQ(message.base_link_linear_velocity[1], 0.4);
  EXPECT_DOUBLE_EQ(message.base_link_angular_velocity[2], 0.8);
  EXPECT_DOUBLE_EQ(message.base_link_quaternion[0], 1.0);
  EXPECT_DOUBLE_EQ(message.base_link_quaternion[1], 0.0);
  EXPECT_DOUBLE_EQ(message.imu_link_quaternion[0], 1.0);
  EXPECT_DOUBLE_EQ(message.imu_sensor_quaternion[0], 0.5);
  EXPECT_DOUBLE_EQ(message.imu_sensor_quaternion[3], 0.5);
  EXPECT_DOUBLE_EQ(message.imu_sensor_linear_acceleration[2], 9.0);
  EXPECT_DOUBLE_EQ(message.imu_sensor_angular_velocity[1], 1.2);
  EXPECT_EQ(message.num_contact_ranges, 0);
  EXPECT_TRUE(message.contact_force.empty());
}

TEST(MujocoLogReplayTest, RobotStatesUseTimestampOrderAfterIterFiltering) {
  const auto dir = MakeTempDir("state_order_filter");
  std::filesystem::create_directories(dir);
  const auto log_path = dir / "states.mcap";
  WriteMcap(log_path,
            {
                MakePayload(1, 10, MakeJointCommand({1.0, 2.0}, {0.1, 0.2},
                                                    {0.0, 0.0})),
                MakePayload(2, 20, MakeJointCommand({2.0, 3.0}, {0.2, 0.3},
                                                    {0.0, 0.0})),
                MakePayload(3, 30, MakeJointCommand({3.0, 4.0}, {0.3, 0.4},
                                                    {0.0, 0.0})),
                MakePayload(4, 40, MakeJointCommand({4.0, 5.0}, {0.4, 0.5},
                                                    {0.0, 0.0})),
            },
            {
                MakeRobotStatePayload(4, 400, {4.0, 5.0}),
                MakeRobotStatePayload(2, 200, {2.0, 3.0}),
                MakeRobotStatePayload(1, 100, {1.0, 2.0}),
                MakeRobotStatePayload(3, 150, {3.0, 4.0}),
            },
            {"joint_a", "joint_b"});

  LoadOptions options = MakeLoadOptions(log_path);
  options.start_iter = 2;
  options.end_iter = 4;
  ReplayLog log = LoadReplayLog(options);

  ASSERT_EQ(log.robot_states.size(), 3U);
  EXPECT_EQ(log.robot_states[0].iter, 3);
  EXPECT_EQ(log.robot_states[0].log_time_ns, 150U);
  EXPECT_EQ(log.robot_states[1].iter, 2);
  EXPECT_EQ(log.robot_states[1].log_time_ns, 200U);
  EXPECT_EQ(log.robot_states[2].iter, 4);
  EXPECT_EQ(log.robot_states[2].log_time_ns, 400U);
  EXPECT_EQ(log.commands.front().iter, 2);
  EXPECT_EQ(log.commands.back().iter, 4);

  std::filesystem::remove_all(dir);
}

TEST(MujocoLogReplayTest, RejectsMismatchedCommandFieldLengths) {
  const auto dir = MakeTempDir("mismatched");
  std::filesystem::create_directories(dir);
  const auto log_path = dir / "commands.mcap";
  WriteMcap(log_path,
            {
                MakePayload(1, 100, MakeJointCommand({1.0, 2.0}, {0.1},
                                                     {0.0, 0.0})),
            },
            {
                MakeRobotStatePayload(1, 101, {1.0, 2.0}),
            },
            {"joint_a", "joint_b"});

  EXPECT_THROW(LoadReplayLog(MakeLoadOptions(log_path)), std::runtime_error);

  std::filesystem::remove_all(dir);
}

TEST(MujocoLogReplayTest, WarnsOnNonzeroTorque) {
  const auto dir = MakeTempDir("torque");
  std::filesystem::create_directories(dir);
  const auto log_path = dir / "commands.mcap";
  WriteMcap(log_path,
            {
                MakePayload(1, 100, MakeJointCommand({1.0, 2.0}, {0.1, 0.2},
                                                     {0.0, 0.5})),
            },
            {
                MakeRobotStatePayload(1, 101, {1.0, 2.0}),
            },
            {"joint_a", "joint_b"});

  ReplayLog log = LoadReplayLog(MakeLoadOptions(log_path));

  EXPECT_TRUE(log.has_nonzero_torque);
  ASSERT_FALSE(log.warnings.empty());

  std::filesystem::remove_all(dir);
}

TEST(MujocoLogReplayTest, RejectsMissingRobotState) {
  const auto dir = MakeTempDir("missing_robot_state");
  std::filesystem::create_directories(dir);
  const auto log_path = dir / "commands.mcap";
  WriteMcap(log_path,
            {
                MakePayload(1, 100, MakeJointCommand({1.0, 2.0}, {0.1, 0.2},
                                                     {0.0, 0.0})),
            },
            {}, {"joint_a", "joint_b"});

  EXPECT_THROW(LoadReplayLog(MakeLoadOptions(log_path)), std::runtime_error);

  std::filesystem::remove_all(dir);
}

TEST(MujocoLogReplayTest, RejectsUnsupportedSchemaVersion) {
  const auto dir = MakeTempDir("schema");
  std::filesystem::create_directories(dir);
  const auto log_path = dir / "commands.mcap";
  WriteMcap(log_path,
            {
                MakePayload(1, 100, MakeJointCommand({1.0, 2.0}, {0.1, 0.2},
                                                     {0.0, 0.0}),
                            1),
            },
            {
                MakeRobotStatePayload(1, 101, {1.0, 2.0},
                                      "base_state_in_world", 1),
            },
            {"joint_a", "joint_b"}, 1);

  EXPECT_THROW(LoadReplayLog(MakeLoadOptions(log_path)), std::runtime_error);

  std::filesystem::remove_all(dir);
}

TEST(MujocoLogReplayTest, DryRunSummaryReportsRobotStateStats) {
  const auto dir = MakeTempDir("summary");
  std::filesystem::create_directories(dir);
  const auto log_path = dir / "commands.mcap";
  WriteMcap(log_path,
            {
                MakePayload(1, 100, MakeJointCommand({1.0, 2.0}, {0.1, 0.2},
                                                     {0.0, 0.0})),
                MakePayload(2, 200, MakeJointCommand({2.0, 3.0}, {0.2, 0.3},
                                                     {0.0, 0.0})),
            },
            {
                MakeRobotStatePayload(1, 110, {1.0, 2.0},
                                      "base_state_in_world"),
                MakeRobotStatePayload(2, 210, {2.0, 3.0},
                                      "simulated_base_state_in_world"),
            },
            {"joint_a", "joint_b"});

  ReplayLog log = LoadReplayLog(MakeLoadOptions(log_path));
  std::ostringstream out;
  PrintDryRunSummary(log, std::nullopt, out);
  const std::string summary = out.str();

  EXPECT_NE(summary.find("robot_state_count: 2"), std::string::npos);
  EXPECT_NE(summary.find("robot_state_joint_count: 2"), std::string::npos);
  EXPECT_NE(summary.find("robot_state_time_range_s:"), std::string::npos);
  EXPECT_NE(summary.find("base_state_in_world"), std::string::npos);
  EXPECT_NE(summary.find("simulated_base_state_in_world"), std::string::npos);

  std::filesystem::remove_all(dir);
}

}  // namespace
}  // namespace mujoco_log_replay
