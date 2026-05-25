#include "rl_tracking_motion/policy_io_mcap_logger.h"

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <filesystem>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <mcap/reader.hpp>
#include <nlohmann/json.hpp>
#include <unistd.h>

namespace runner {
namespace {

std::filesystem::path MakeTempDir(std::string_view test_name) {
  const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  return std::filesystem::temp_directory_path() /
         ("rl_tracking_motion_" + std::string(test_name) + "_" +
          std::to_string(getpid()) + "_" + std::to_string(now));
}

class ScopedEnv {
public:
  ScopedEnv(const char *name, const std::string &value) : name_(name) {
    const char *previous = std::getenv(name_.c_str());
    if (previous != nullptr) {
      previous_ = previous;
    }
    setenv(name_.c_str(), value.c_str(), 1);
  }

  ~ScopedEnv() {
    if (previous_.has_value()) {
      setenv(name_.c_str(), previous_->c_str(), 1);
    } else {
      unsetenv(name_.c_str());
    }
  }

private:
  std::string name_;
  std::optional<std::string> previous_;
};

Eigen::MatrixXf MakeMatrix(int rows, int cols, float start) {
  Eigen::MatrixXf matrix(rows, cols);
  float value = start;
  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < cols; ++col) {
      matrix(row, col) = value;
      value += 1.0F;
    }
  }
  return matrix;
}

PolicyOutputs MakeOutputs(int num_actions) {
  PolicyOutputs outputs;
  outputs.actions = Eigen::VectorXf::LinSpaced(num_actions, 0.1F, 0.4F);
  outputs.joint_pos = Eigen::VectorXf::LinSpaced(num_actions, 1.0F, 4.0F);
  outputs.joint_vel = Eigen::VectorXf::LinSpaced(num_actions, 5.0F, 8.0F);
  outputs.body_pos_w = MakeMatrix(3, 3, 10.0F);
  outputs.body_quat_w = MakeMatrix(3, 4, 20.0F);
  outputs.body_lin_vel_w = MakeMatrix(3, 3, 40.0F);
  outputs.body_ang_vel_w = MakeMatrix(3, 3, 50.0F);
  return outputs;
}

PolicyIoMcapLoggerConfig MakeConfig(const std::filesystem::path &dir,
                                    bool enabled = true) {
  PolicyIoMcapLoggerConfig config;
  config.enabled = enabled;
  config.directory = dir;
  config.runner = "rl_tracking_motion_runner";
  config.param_tag = "test_motion";
  config.policy_file = "test_motion/policies/policy.mnn";
  config.joint_names = {"joint_a", "joint_b", "joint_c", "joint_d"};
  config.command_joint_names = {"joint_a", "joint_b", "joint_c", "joint_d"};
  config.state_joint_names = config.command_joint_names;
  config.observation_names = {"joint_pos", "motion_phase"};
  config.observation_history_lengths = {1, 3};
  config.observation_layout = {
      {"joint_pos", 1, 4, 0, 4},
      {"motion_phase", 3, 2, 4, 6},
  };
  config.num_actions = 4;
  return config;
}

PolicyIoJointCommand MakeJointCommand(int num_joints) {
  PolicyIoJointCommand command;
  command.position = Eigen::VectorXd::LinSpaced(num_joints, 0.1, 0.4);
  command.velocity = Eigen::VectorXd::LinSpaced(num_joints, 1.1, 1.4);
  command.feed_forward_torque =
      Eigen::VectorXd::LinSpaced(num_joints, 2.1, 2.4);
  command.torque = Eigen::VectorXd::Zero(num_joints);
  command.stiffness = Eigen::VectorXd::LinSpaced(num_joints, 3.1, 3.4);
  command.damping = Eigen::VectorXd::LinSpaced(num_joints, 4.1, 4.4);
  return command;
}

PolicyIoJointState MakeJointState(int num_joints, double start) {
  PolicyIoJointState state;
  state.position = Eigen::VectorXd::LinSpaced(num_joints, start, start + 0.3);
  state.velocity =
      Eigen::VectorXd::LinSpaced(num_joints, start + 1.0, start + 1.3);
  state.torque =
      Eigen::VectorXd::LinSpaced(num_joints, start + 2.0, start + 2.3);
  return state;
}

PolicyIoLinkState MakeLinkState(double start) {
  PolicyIoLinkState state;
  state.position = Eigen::Vector3d(start, start + 1.0, start + 2.0);
  state.quaternion =
      Eigen::Quaterniond(start + 3.0, start + 4.0, start + 5.0, start + 6.0);
  state.linear_velocity =
      Eigen::Vector3d(start + 7.0, start + 8.0, start + 9.0);
  state.angular_velocity =
      Eigen::Vector3d(start + 10.0, start + 11.0, start + 12.0);
  state.linear_acceleration =
      Eigen::Vector3d(start + 13.0, start + 14.0, start + 15.0);
  state.angular_acceleration =
      Eigen::Vector3d(start + 16.0, start + 17.0, start + 18.0);
  return state;
}

PolicyIoRobotState MakeRobotState(int num_joints) {
  PolicyIoRobotState state;
  state.joint_state = MakeJointState(num_joints, 10.0);
  state.motor_state = MakeJointState(num_joints, 20.0);
  state.imu.quaternion = Eigen::Quaterniond(1.0, 0.1, 0.2, 0.3);
  state.imu.rpy = Eigen::Vector3d(0.4, 0.5, 0.6);
  state.imu.linear_acceleration = Eigen::Vector3d(0.7, 0.8, 0.9);
  state.imu.angular_velocity = Eigen::Vector3d(1.1, 1.2, 1.3);
  state.base_state_in_world = MakeLinkState(30.0);
  state.simulated_base_state_in_world = MakeLinkState(50.0);
  state.selected_anchor_state = MakeLinkState(70.0);
  state.selected_anchor_source = "simulated_base_state_in_world";
  return state;
}

mcap::KeyValueMap ReadPolicyIoMetadata(const std::filesystem::path &file_path) {
  std::ifstream stream(file_path, std::ios::binary);
  EXPECT_TRUE(stream.good());

  mcap::FileStreamReader file_reader(stream);
  mcap::RecordReader record_reader(file_reader, sizeof(mcap::Magic),
                                   file_reader.size() - sizeof(mcap::Magic));
  while (const auto record = record_reader.next()) {
    if (record->opcode != mcap::OpCode::Metadata) {
      continue;
    }
    mcap::Metadata metadata;
    auto status = mcap::McapReader::ParseMetadata(*record, &metadata);
    EXPECT_TRUE(status.ok()) << status.message;
    if (metadata.name == "rl_tracking_motion_policy_io_metadata") {
      return metadata.metadata;
    }
  }
  return {};
}

void ExpectPolicyMessageShape(const nlohmann::json &message, int obs_size,
                              int num_actions) {
  EXPECT_EQ(message.at("schema_version"), 2);
  EXPECT_EQ(message.at("runner"), "rl_tracking_motion_runner");
  EXPECT_EQ(message.at("param_tag"), "test_motion");
  EXPECT_EQ(message.at("policy_file"), "test_motion/policies/policy.mnn");
  EXPECT_TRUE(message.contains("log_time_ns"));

  ASSERT_TRUE(message.contains("inputs"));
  EXPECT_EQ(message.at("inputs").at("obs").size(),
            static_cast<size_t>(obs_size));
  EXPECT_EQ(message.at("inputs").at("time_step").size(), 1U);

  ASSERT_TRUE(message.contains("outputs"));
  const auto &outputs = message.at("outputs");
  EXPECT_EQ(outputs.at("actions").size(), static_cast<size_t>(num_actions));
  EXPECT_EQ(outputs.at("joint_pos").size(), static_cast<size_t>(num_actions));
  EXPECT_EQ(outputs.at("joint_vel").size(), static_cast<size_t>(num_actions));
  EXPECT_EQ(outputs.at("body_pos_w").size(), 9U);
  EXPECT_EQ(outputs.at("body_quat_w").size(), 12U);
  EXPECT_EQ(outputs.at("body_lin_vel_w").size(), 9U);
  EXPECT_EQ(outputs.at("body_ang_vel_w").size(), 9U);
}

template <typename RecordFn>
void RecordUntilEnqueued(PolicyIoMcapLogger &logger, RecordFn record) {
  const uint64_t before = logger.GetStats().enqueued_messages;
  for (int attempt = 0; attempt < 1000; ++attempt) {
    record();
    if (logger.GetStats().enqueued_messages > before) {
      return;
    }
    std::this_thread::yield();
  }
  ADD_FAILURE() << "failed to enqueue policy I/O record";
}

TEST(PolicyIoMcapLoggerTest, WritesReferenceAndActionMessages) {
  const std::filesystem::path dir = MakeTempDir("writes");
  std::filesystem::remove_all(dir);

  PolicyIoMcapLogger logger;
  logger.Start(MakeConfig(dir));
  ASSERT_TRUE(logger.IsEnabled());

  const Eigen::VectorXf obs = Eigen::VectorXf::LinSpaced(6, -1.0F, 1.0F);
  const PolicyOutputs outputs = MakeOutputs(4);
  const PolicyIoJointCommand command = MakeJointCommand(4);
  const PolicyIoRobotState robot_state = MakeRobotState(4);
  const PolicyIoInvocationContext context{7, 11, 0.02};
  RecordUntilEnqueued(logger,
                      [&] { logger.RecordRobotState(context, robot_state); });
  RecordUntilEnqueued(
      logger, [&] { logger.RecordReference(context, obs, 11, outputs); });
  RecordUntilEnqueued(logger,
                      [&] { logger.RecordAction(context, obs, 11, outputs); });
  RecordUntilEnqueued(logger, [&] {
    logger.RecordJointCommandFeedback(context, command);
  });

  const std::filesystem::path file_path = logger.FilePath();
  logger.Close();
  const PolicyIoMcapLoggerStats stats = logger.GetStats();
  EXPECT_EQ(stats.enqueued_messages, 4U);
  EXPECT_EQ(stats.written_messages, 4U);
  EXPECT_EQ(stats.dropped_queue_full, 0U);
  EXPECT_EQ(stats.dropped_after_error, 0U);
  EXPECT_EQ(stats.write_errors, 0U);
  EXPECT_GE(stats.max_queue_depth, 1U);
  ASSERT_TRUE(std::filesystem::exists(file_path));

  mcap::McapReader reader;
  auto status = reader.open(file_path.string());
  ASSERT_TRUE(status.ok()) << status.message;
  status = reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);
  ASSERT_TRUE(status.ok()) << status.message;

  const auto schemas = reader.schemas();
  ASSERT_EQ(schemas.size(), 1U);
  const auto schema = schemas.begin()->second;
  ASSERT_NE(schema, nullptr);
  EXPECT_EQ(schema->encoding, "jsonschema");

  const auto metadata = ReadPolicyIoMetadata(file_path);
  ASSERT_FALSE(metadata.empty());
  EXPECT_EQ(metadata.at("schema_version"), "2");
  EXPECT_EQ(metadata.at("robot_state_topic"),
            "/rl_tracking_motion/robot_state");
  EXPECT_EQ(nlohmann::json::parse(metadata.at("state_joint_names")).size(),
            4U);
  const auto robot_state_shapes =
      nlohmann::json::parse(metadata.at("robot_state_shapes"));
  EXPECT_EQ(robot_state_shapes.at("joint_state")
                .at("position")
                .at(0)
                .get<int>(),
            4);
  EXPECT_EQ(robot_state_shapes.at("imu")
                .at("quaternion_wxyz")
                .at(0)
                .get<int>(),
            4);

  const auto channels = reader.channels();
  ASSERT_EQ(channels.size(), 4U);
  std::set<std::string> topics;
  for (const auto &[_, channel] : channels) {
    ASSERT_NE(channel, nullptr);
    EXPECT_EQ(channel->messageEncoding, "json");
    topics.insert(channel->topic);
  }
  EXPECT_TRUE(topics.contains("/rl_tracking_motion/policy/reference"));
  EXPECT_TRUE(topics.contains("/rl_tracking_motion/policy/action"));
  EXPECT_TRUE(topics.contains("/hardware/joint_command_feedback"));
  EXPECT_TRUE(topics.contains("/rl_tracking_motion/robot_state"));

  ASSERT_TRUE(reader.statistics().has_value());
  EXPECT_EQ(reader.statistics()->metadataCount, 1U);
  EXPECT_EQ(reader.statistics()->messageCount, 4U);

  std::map<std::string, int> counts_by_invocation;
  std::map<std::string, int> counts_by_topic;
  for (const auto &message_view : reader.readMessages()) {
    const auto &message = message_view.message;
    const auto *data = reinterpret_cast<const char *>(message.data);
    nlohmann::json payload =
        nlohmann::json::parse(std::string(data, data + message.dataSize));
    const std::string invocation =
        payload.at("invocation").get<std::string>();
    if (invocation == "joint_command_feedback") {
      EXPECT_EQ(payload.at("schema_version"), 2);
      const auto &joint_command = payload.at("joint_command_feedback");
      EXPECT_EQ(joint_command.at("position").size(), 4U);
      EXPECT_EQ(joint_command.at("velocity").size(), 4U);
      EXPECT_EQ(joint_command.at("feed_forward_torque").size(), 4U);
      EXPECT_EQ(joint_command.at("torque").size(), 4U);
      EXPECT_EQ(joint_command.at("stiffness").size(), 4U);
      EXPECT_EQ(joint_command.at("damping").size(), 4U);
      EXPECT_EQ(payload.at("topic").get<std::string>(),
                "/hardware/joint_command_feedback");
    } else if (invocation == "robot_state") {
      EXPECT_EQ(payload.at("schema_version"), 2);
      EXPECT_EQ(payload.at("topic").get<std::string>(),
                "/rl_tracking_motion/robot_state");
      const auto &state = payload.at("robot_state");
      EXPECT_EQ(state.at("joint_state").at("position").size(), 4U);
      EXPECT_EQ(state.at("joint_state").at("velocity").size(), 4U);
      EXPECT_EQ(state.at("joint_state").at("torque").size(), 4U);
      EXPECT_DOUBLE_EQ(
          state.at("joint_state").at("position").at(0).get<double>(), 10.0);
      EXPECT_EQ(state.at("motor_state").at("position").size(), 4U);
      EXPECT_DOUBLE_EQ(
          state.at("motor_state").at("torque").at(3).get<double>(), 22.3);
      EXPECT_EQ(state.at("imu").at("quaternion_wxyz").size(), 4U);
      EXPECT_DOUBLE_EQ(
          state.at("imu").at("quaternion_wxyz").at(0).get<double>(), 1.0);
      EXPECT_EQ(state.at("base_state_in_world").at("position").size(), 3U);
      EXPECT_DOUBLE_EQ(state.at("base_state_in_world")
                           .at("position")
                           .at(0)
                           .get<double>(),
                       30.0);
      EXPECT_EQ(state.at("simulated_base_state_in_world")
                    .at("quaternion_wxyz")
                    .size(),
                4U);
      EXPECT_DOUBLE_EQ(state.at("selected_anchor_state")
                           .at("angular_acceleration")
                           .at(2)
                           .get<double>(),
                       88.0);
      EXPECT_EQ(state.at("selected_anchor_source").get<std::string>(),
                "simulated_base_state_in_world");
    } else {
      ExpectPolicyMessageShape(payload, static_cast<int>(obs.size()), 4);
    }
    counts_by_invocation[payload.at("invocation").get<std::string>()]++;
    counts_by_topic[message_view.channel->topic]++;
  }

  EXPECT_EQ(counts_by_invocation["reference"], 1);
  EXPECT_EQ(counts_by_invocation["action"], 1);
  EXPECT_EQ(counts_by_invocation["joint_command_feedback"], 1);
  EXPECT_EQ(counts_by_invocation["robot_state"], 1);
  EXPECT_EQ(counts_by_topic["/rl_tracking_motion/policy/reference"], 1);
  EXPECT_EQ(counts_by_topic["/rl_tracking_motion/policy/action"], 1);
  EXPECT_EQ(counts_by_topic["/hardware/joint_command_feedback"], 1);
  EXPECT_EQ(counts_by_topic["/rl_tracking_motion/robot_state"], 1);

  reader.close();
  std::filesystem::remove_all(dir);
}

TEST(PolicyIoMcapLoggerTest, DisabledLoggerDoesNotCreateFile) {
  const std::filesystem::path dir = MakeTempDir("disabled");
  std::filesystem::remove_all(dir);

  PolicyIoMcapLogger logger;
  logger.Start(MakeConfig(dir, false));
  EXPECT_FALSE(logger.IsEnabled());

  const Eigen::VectorXf obs = Eigen::VectorXf::LinSpaced(6, -1.0F, 1.0F);
  const PolicyOutputs outputs = MakeOutputs(4);
  const PolicyIoJointCommand command = MakeJointCommand(4);
  const PolicyIoRobotState robot_state = MakeRobotState(4);
  const PolicyIoInvocationContext context{1, 2, 0.02};
  EXPECT_NO_THROW(logger.RecordRobotState(context, robot_state));
  EXPECT_NO_THROW(logger.RecordReference(context, obs, 2, outputs));
  EXPECT_NO_THROW(logger.RecordAction(context, obs, 2, outputs));
  EXPECT_NO_THROW(logger.RecordJointCommandFeedback(context, command));
  logger.Close();

  const PolicyIoMcapLoggerStats stats = logger.GetStats();
  EXPECT_EQ(stats.enqueued_messages, 0U);
  EXPECT_EQ(stats.written_messages, 0U);
  EXPECT_EQ(stats.dropped_queue_full, 0U);
  EXPECT_EQ(stats.dropped_lock_busy, 0U);
  EXPECT_EQ(stats.dropped_after_error, 0U);
  EXPECT_EQ(stats.write_errors, 0U);
  EXPECT_FALSE(std::filesystem::exists(dir));
}

TEST(PolicyIoMcapLoggerTest, CloseDrainsPendingAsyncMessages) {
  const std::filesystem::path dir = MakeTempDir("drain");
  std::filesystem::remove_all(dir);

  PolicyIoMcapLogger logger;
  logger.Start(MakeConfig(dir));
  ASSERT_TRUE(logger.IsEnabled());

  const PolicyOutputs outputs = MakeOutputs(4);
  const PolicyIoInvocationContext context{3, 5, 0.02};
  for (int i = 0; i < 32; ++i) {
    Eigen::VectorXf obs = Eigen::VectorXf::LinSpaced(6, -1.0F, 1.0F);
    obs(0) = static_cast<float>(i);
    RecordUntilEnqueued(logger, [&] {
      logger.RecordReference(context, obs, i, outputs);
    });
  }

  const std::filesystem::path file_path = logger.FilePath();
  logger.Close();
  const PolicyIoMcapLoggerStats stats = logger.GetStats();
  EXPECT_EQ(stats.enqueued_messages, 32U);
  EXPECT_EQ(stats.written_messages, 32U);
  EXPECT_EQ(stats.dropped_queue_full, 0U);
  EXPECT_EQ(stats.dropped_after_error, 0U);
  EXPECT_EQ(stats.write_errors, 0U);

  mcap::McapReader reader;
  auto status = reader.open(file_path.string());
  ASSERT_TRUE(status.ok()) << status.message;
  status = reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);
  ASSERT_TRUE(status.ok()) << status.message;
  ASSERT_TRUE(reader.statistics().has_value());
  EXPECT_EQ(reader.statistics()->messageCount, 32U);

  std::set<int> time_steps;
  for (const auto &message_view : reader.readMessages()) {
    const auto &message = message_view.message;
    const auto *data = reinterpret_cast<const char *>(message.data);
    const nlohmann::json payload =
        nlohmann::json::parse(std::string(data, data + message.dataSize));
    EXPECT_EQ(payload.at("invocation").get<std::string>(), "reference");
    time_steps.insert(static_cast<int>(
        payload.at("inputs").at("time_step").at(0).get<float>()));
  }
  EXPECT_EQ(time_steps.size(), 32U);
  EXPECT_TRUE(time_steps.contains(0));
  EXPECT_TRUE(time_steps.contains(31));

  reader.close();
  std::filesystem::remove_all(dir);
}

TEST(PolicyIoMcapLoggerTest, FullQueueDropsNewMessages) {
  const std::filesystem::path dir = MakeTempDir("queue_full");
  std::filesystem::remove_all(dir);

  PolicyIoMcapLoggerConfig config = MakeConfig(dir);
  config.max_pending_messages = 0;

  PolicyIoMcapLogger logger;
  logger.Start(config);
  ASSERT_TRUE(logger.IsEnabled());

  const Eigen::VectorXf obs = Eigen::VectorXf::LinSpaced(6, -1.0F, 1.0F);
  const PolicyOutputs outputs = MakeOutputs(4);
  const PolicyIoInvocationContext context{1, 2, 0.02};
  for (int i = 0; i < 16; ++i) {
    logger.RecordReference(context, obs, i, outputs);
  }

  const std::filesystem::path file_path = logger.FilePath();
  logger.Close();
  const PolicyIoMcapLoggerStats stats = logger.GetStats();
  EXPECT_EQ(stats.enqueued_messages, 0U);
  EXPECT_EQ(stats.written_messages, 0U);
  EXPECT_GT(stats.dropped_queue_full, 0U);
  EXPECT_EQ(stats.dropped_after_error, 0U);
  EXPECT_EQ(stats.write_errors, 0U);

  mcap::McapReader reader;
  auto status = reader.open(file_path.string());
  ASSERT_TRUE(status.ok()) << status.message;
  status = reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);
  ASSERT_TRUE(status.ok()) << status.message;
  ASSERT_TRUE(reader.statistics().has_value());
  EXPECT_EQ(reader.statistics()->messageCount, 0U);
  reader.close();

  std::filesystem::remove_all(dir);
}

TEST(PolicyIoMcapLoggerTest, StartErrorStopsAcceptingNewMessages) {
  PolicyIoMcapLoggerConfig config = MakeConfig("");
  PolicyIoMcapLogger logger;
  logger.Start(config);
  EXPECT_FALSE(logger.IsEnabled());

  const Eigen::VectorXf obs = Eigen::VectorXf::LinSpaced(6, -1.0F, 1.0F);
  const PolicyOutputs outputs = MakeOutputs(4);
  const PolicyIoInvocationContext context{1, 2, 0.02};
  EXPECT_NO_THROW(logger.RecordReference(context, obs, 2, outputs));
  logger.Close();

  const PolicyIoMcapLoggerStats stats = logger.GetStats();
  EXPECT_EQ(stats.enqueued_messages, 0U);
  EXPECT_EQ(stats.written_messages, 0U);
  EXPECT_EQ(stats.write_errors, 1U);
  EXPECT_EQ(stats.dropped_after_error, 1U);
}

TEST(PolicyIoMcapLoggerTest, RelativeDirectoryUsesEngineAiRoboticsDir) {
  const std::filesystem::path root = MakeTempDir("relative_root");
  std::filesystem::remove_all(root);
  const ScopedEnv env("ENGINEAI_ROBOTICS_DIR", root.string());

  PolicyIoMcapLogger logger;
  logger.Start(MakeConfig("logs"));
  ASSERT_TRUE(logger.IsEnabled());

  const std::filesystem::path file_path = logger.FilePath();
  logger.Close();

  EXPECT_EQ(file_path.parent_path(), root / "logs");
  EXPECT_TRUE(std::filesystem::exists(file_path));

  std::filesystem::remove_all(root);
}

} // namespace
} // namespace runner
