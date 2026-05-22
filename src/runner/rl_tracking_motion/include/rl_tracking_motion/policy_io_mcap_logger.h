#pragma once

#include <Eigen/Dense>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace runner {

struct PolicyOutputs {
  Eigen::VectorXf actions;
  Eigen::VectorXf joint_pos;
  Eigen::VectorXf joint_vel;
  Eigen::MatrixXf body_pos_w;
  Eigen::MatrixXf body_quat_w;
  Eigen::MatrixXf body_lin_vel_w;
  Eigen::MatrixXf body_ang_vel_w;
};

struct PolicyIoObservationTermLayout {
  std::string name;
  int history_length = 0;
  int value_dim = 0;
  int flattened_offset = 0;
  int flattened_dim = 0;
};

struct PolicyIoMcapLoggerConfig {
  bool enabled = true;
  std::filesystem::path directory;
  std::string runner;
  std::string param_tag;
  std::string policy_file;
  std::vector<std::string> joint_names;
  std::vector<std::string> command_joint_names;
  std::vector<std::string> observation_names;
  std::vector<int> observation_history_lengths;
  std::vector<PolicyIoObservationTermLayout> observation_layout;
  int num_actions = 0;
  std::string joint_command_feedback_topic = "/hardware/joint_command_feedback";
};

struct PolicyIoInvocationContext {
  int iter = 0;
  int runner_time_step = 0;
  double runner_period_s = 0.0;
};

struct PolicyIoJointCommand {
  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd feed_forward_torque;
  Eigen::VectorXd torque;
  Eigen::VectorXd stiffness;
  Eigen::VectorXd damping;
};

class PolicyIoMcapLogger {
public:
  PolicyIoMcapLogger();
  ~PolicyIoMcapLogger();

  PolicyIoMcapLogger(const PolicyIoMcapLogger &) = delete;
  PolicyIoMcapLogger &operator=(const PolicyIoMcapLogger &) = delete;

  void Start(const PolicyIoMcapLoggerConfig &config);
  void Close();

  void RecordReference(const PolicyIoInvocationContext &context,
                       const Eigen::VectorXf &obs, int time_step,
                       const PolicyOutputs &outputs);
  void RecordAction(const PolicyIoInvocationContext &context,
                    const Eigen::VectorXf &obs, int time_step,
                    const PolicyOutputs &outputs);
  void RecordJointCommandFeedback(const PolicyIoInvocationContext &context,
                                  const PolicyIoJointCommand &command);

  bool IsEnabled() const;
  const std::filesystem::path &FilePath() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace runner
