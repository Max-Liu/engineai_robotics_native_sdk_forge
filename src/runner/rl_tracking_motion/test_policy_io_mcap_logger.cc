#include "rl_tracking_motion/policy_io_mcap_logger.h"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <string_view>
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

void ExpectPolicyMessageShape(const nlohmann::json &message, int obs_size,
                              int num_actions) {
  EXPECT_EQ(message.at("schema_version"), 1);
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

TEST(PolicyIoMcapLoggerTest, WritesReferenceAndActionMessages) {
  const std::filesystem::path dir = MakeTempDir("writes");
  std::filesystem::remove_all(dir);

  PolicyIoMcapLogger logger;
  logger.Start(MakeConfig(dir));
  ASSERT_TRUE(logger.IsEnabled());

  const Eigen::VectorXf obs = Eigen::VectorXf::LinSpaced(6, -1.0F, 1.0F);
  const PolicyOutputs outputs = MakeOutputs(4);
  const PolicyIoJointCommand command = MakeJointCommand(4);
  const PolicyIoInvocationContext context{7, 11, 0.02};
  logger.RecordReference(context, obs, 11, outputs);
  logger.RecordAction(context, obs, 11, outputs);
  logger.RecordJointCommandFeedback(context, command);

  const std::filesystem::path file_path = logger.FilePath();
  logger.Close();
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

  const auto channels = reader.channels();
  ASSERT_EQ(channels.size(), 3U);
  std::set<std::string> topics;
  for (const auto &[_, channel] : channels) {
    ASSERT_NE(channel, nullptr);
    EXPECT_EQ(channel->messageEncoding, "json");
    topics.insert(channel->topic);
  }
  EXPECT_TRUE(topics.contains("/rl_tracking_motion/policy/reference"));
  EXPECT_TRUE(topics.contains("/rl_tracking_motion/policy/action"));
  EXPECT_TRUE(topics.contains("/hardware/joint_command_feedback"));

  ASSERT_TRUE(reader.statistics().has_value());
  EXPECT_EQ(reader.statistics()->metadataCount, 1U);
  EXPECT_EQ(reader.statistics()->messageCount, 3U);

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
      const auto &joint_command = payload.at("joint_command_feedback");
      EXPECT_EQ(joint_command.at("position").size(), 4U);
      EXPECT_EQ(joint_command.at("velocity").size(), 4U);
      EXPECT_EQ(joint_command.at("feed_forward_torque").size(), 4U);
      EXPECT_EQ(joint_command.at("torque").size(), 4U);
      EXPECT_EQ(joint_command.at("stiffness").size(), 4U);
      EXPECT_EQ(joint_command.at("damping").size(), 4U);
      EXPECT_EQ(payload.at("topic").get<std::string>(),
                "/hardware/joint_command_feedback");
    } else {
      ExpectPolicyMessageShape(payload, static_cast<int>(obs.size()), 4);
    }
    counts_by_invocation[payload.at("invocation").get<std::string>()]++;
    counts_by_topic[message_view.channel->topic]++;
  }

  EXPECT_EQ(counts_by_invocation["reference"], 1);
  EXPECT_EQ(counts_by_invocation["action"], 1);
  EXPECT_EQ(counts_by_invocation["joint_command_feedback"], 1);
  EXPECT_EQ(counts_by_topic["/rl_tracking_motion/policy/reference"], 1);
  EXPECT_EQ(counts_by_topic["/rl_tracking_motion/policy/action"], 1);
  EXPECT_EQ(counts_by_topic["/hardware/joint_command_feedback"], 1);

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
  const PolicyIoInvocationContext context{1, 2, 0.02};
  EXPECT_NO_THROW(logger.RecordReference(context, obs, 2, outputs));
  EXPECT_NO_THROW(logger.RecordAction(context, obs, 2, outputs));
  EXPECT_NO_THROW(logger.RecordJointCommandFeedback(context, command));
  logger.Close();

  EXPECT_FALSE(std::filesystem::exists(dir));
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
