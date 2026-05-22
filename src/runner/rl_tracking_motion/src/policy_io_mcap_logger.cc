#include "rl_tracking_motion/policy_io_mcap_logger.h"

#include <chrono>
#include <cctype>
#include <cstddef>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string_view>

#include <glog/logging.h>
#include <mcap/writer.hpp>
#include <nlohmann/json.hpp>
#include <unistd.h>

namespace runner {
namespace {

constexpr std::string_view kProfile = "json";
constexpr std::string_view kMessageEncoding = "json";
constexpr std::string_view kSchemaEncoding = "jsonschema";
constexpr std::string_view kSchemaName =
    "engineai.rl_tracking_motion.PolicyIo";
constexpr std::string_view kReferenceTopic =
    "/rl_tracking_motion/policy/reference";
constexpr std::string_view kActionTopic = "/rl_tracking_motion/policy/action";
constexpr std::string_view kJointCommandFeedbackInvocation =
    "joint_command_feedback";
constexpr std::string_view kMetadataName =
    "rl_tracking_motion_policy_io_metadata";

uint64_t NowNs() {
  const auto now = std::chrono::system_clock::now();
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          now.time_since_epoch())
          .count());
}

std::string TimestampForFilename() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time{};
  localtime_r(&time, &local_time);

  std::ostringstream out;
  out << std::put_time(&local_time, "%Y%m%d_%H%M%S");
  return out.str();
}

std::string SanitizeForFilename(std::string value) {
  for (char &ch : value) {
    const auto uch = static_cast<unsigned char>(ch);
    if (!std::isalnum(uch) && ch != '-' && ch != '_') {
      ch = '_';
    }
  }
  if (value.empty()) {
    return "unknown";
  }
  return value;
}

std::filesystem::path
ResolveLogDirectory(const std::filesystem::path &directory) {
  if (directory.is_absolute()) {
    return directory;
  }

  const char *robotics_dir = std::getenv("ENGINEAI_ROBOTICS_DIR");
  if (robotics_dir != nullptr && robotics_dir[0] != '\0') {
    return std::filesystem::path(robotics_dir) / directory;
  }

  return std::filesystem::current_path() / directory;
}

nlohmann::json VectorToJson(const Eigen::VectorXf &value) {
  nlohmann::json out = nlohmann::json::array();
  for (Eigen::Index i = 0; i < value.size(); ++i) {
    out.push_back(value(i));
  }
  return out;
}

nlohmann::json VectorToJson(const Eigen::VectorXd &value) {
  nlohmann::json out = nlohmann::json::array();
  for (Eigen::Index i = 0; i < value.size(); ++i) {
    out.push_back(value(i));
  }
  return out;
}

nlohmann::json MatrixToRowMajorJson(const Eigen::MatrixXf &value) {
  nlohmann::json out = nlohmann::json::array();
  for (Eigen::Index row = 0; row < value.rows(); ++row) {
    for (Eigen::Index col = 0; col < value.cols(); ++col) {
      out.push_back(value(row, col));
    }
  }
  return out;
}

nlohmann::json OutputShapesJson(int num_actions) {
  return {
      {"actions", {num_actions}},
      {"joint_pos", {num_actions}},
      {"joint_vel", {num_actions}},
      {"body_pos_w", {3, 3}},
      {"body_quat_w", {3, 4}},
      {"body_lin_vel_w", {3, 3}},
      {"body_ang_vel_w", {3, 3}},
      {"matrix_storage_order", "row_major_flatten"},
  };
}

nlohmann::json JointCommandShapesJson(int num_joints) {
  return {
      {"position", {num_joints}},
      {"velocity", {num_joints}},
      {"feed_forward_torque", {num_joints}},
      {"torque", {num_joints}},
      {"stiffness", {num_joints}},
      {"damping", {num_joints}},
  };
}

nlohmann::json ObservationLayoutJson(
    const std::vector<PolicyIoObservationTermLayout> &layout) {
  nlohmann::json out = nlohmann::json::array();
  for (const auto &term : layout) {
    out.push_back({
        {"name", term.name},
        {"history_length", term.history_length},
        {"value_dim", term.value_dim},
        {"flattened_offset", term.flattened_offset},
        {"flattened_dim", term.flattened_dim},
    });
  }
  return out;
}

nlohmann::json BuildSchemaJson() {
  const nlohmann::json number_array = {
      {"type", "array"},
      {"items", {{"type", "number"}}},
  };

  return {
      {"$schema", "http://json-schema.org/draft-07/schema#"},
      {"title", "RL Tracking Motion Policy I/O"},
      {"type", "object"},
      {"required",
       {"schema_version", "runner", "param_tag", "policy_file", "invocation",
        "iter", "runner_time_step", "runner_period_s", "log_time_ns"}},
      {"properties",
       {
           {"schema_version", {{"type", "integer"}, {"const", 1}}},
           {"runner", {{"type", "string"}}},
           {"param_tag", {{"type", "string"}}},
           {"policy_file", {{"type", "string"}}},
           {"invocation", {{"type", "string"}}},
           {"iter", {{"type", "integer"}}},
           {"runner_time_step", {{"type", "integer"}}},
           {"runner_period_s", {{"type", "number"}}},
           {"log_time_ns", {{"type", "integer"}}},
           {"inputs",
            {{"type", "object"},
             {"required", {"obs", "time_step"}},
             {"properties",
              {
                  {"obs", number_array},
                  {"time_step", number_array},
              }}}},
           {"outputs",
            {{"type", "object"},
             {"required",
              {"actions", "joint_pos", "joint_vel", "body_pos_w",
               "body_quat_w", "body_lin_vel_w", "body_ang_vel_w"}},
             {"properties",
              {
                  {"actions", number_array},
                  {"joint_pos", number_array},
                  {"joint_vel", number_array},
                  {"body_pos_w", number_array},
                  {"body_quat_w", number_array},
                  {"body_lin_vel_w", number_array},
                  {"body_ang_vel_w", number_array},
              }}}},
           {"joint_command_feedback",
            {{"type", "object"},
             {"properties",
              {
                  {"position", number_array},
                  {"velocity", number_array},
                  {"feed_forward_torque", number_array},
                  {"torque", number_array},
                  {"stiffness", number_array},
                  {"damping", number_array},
              }}}},
       }},
  };
}

} // namespace

class PolicyIoMcapLogger::Impl {
public:
  void Start(const PolicyIoMcapLoggerConfig &config) {
    Close();
    config_ = config;
    file_path_.clear();
    reference_sequence_ = 0;
    action_sequence_ = 0;
    joint_command_feedback_sequence_ = 0;
    metadata_written_ = false;

    if (!config_.enabled) {
      return;
    }

    try {
      if (config_.directory.empty()) {
        throw std::runtime_error("policy_io_mcap_dir is empty");
      }
      const auto resolved_directory = ResolveLogDirectory(config_.directory);
      std::filesystem::create_directories(resolved_directory);
      file_path_ = resolved_directory / BuildFileName();

      mcap::McapWriterOptions options(kProfile);
      options.noChunking = true;
      auto status = writer_.open(file_path_.string(), options);
      if (!status.ok()) {
        DisableAfterError("open", status.message);
        return;
      }

      open_ = true;
      enabled_ = true;
      RegisterSchemaAndChannels();
      LOG(INFO) << "[PolicyIoMcapLogger] Writing RL tracking policy I/O to "
                << file_path_;
    } catch (const std::exception &error) {
      DisableAfterError("start", error.what());
    }
  }

  void Close() {
    if (!open_) {
      enabled_ = false;
      return;
    }

    try {
      if (!metadata_written_) {
        WriteMetadata();
      }
      if (!open_) {
        return;
      }
      writer_.close();
    } catch (const std::exception &error) {
      LOG(ERROR) << "[PolicyIoMcapLogger] Failed to close MCAP file "
                 << file_path_ << ": " << error.what();
      writer_.terminate();
    }

    open_ = false;
    enabled_ = false;
  }

  void Record(std::string_view invocation,
              const PolicyIoInvocationContext &context,
              const Eigen::VectorXf &obs, int time_step,
              const PolicyOutputs &outputs) {
    if (!enabled_ || !open_) {
      return;
    }

    try {
      const uint64_t now_ns = NowNs();
      nlohmann::json payload = {
          {"schema_version", 1},
          {"runner", config_.runner},
          {"param_tag", config_.param_tag},
          {"policy_file", config_.policy_file},
          {"invocation", invocation},
          {"iter", context.iter},
          {"runner_time_step", context.runner_time_step},
          {"runner_period_s", context.runner_period_s},
          {"log_time_ns", now_ns},
          {"inputs",
           {
               {"obs", VectorToJson(obs)},
               {"time_step", {static_cast<float>(time_step)}},
           }},
          {"outputs",
           {
               {"actions", VectorToJson(outputs.actions)},
               {"joint_pos", VectorToJson(outputs.joint_pos)},
               {"joint_vel", VectorToJson(outputs.joint_vel)},
               {"body_pos_w", MatrixToRowMajorJson(outputs.body_pos_w)},
               {"body_quat_w", MatrixToRowMajorJson(outputs.body_quat_w)},
               {"body_lin_vel_w",
                MatrixToRowMajorJson(outputs.body_lin_vel_w)},
               {"body_ang_vel_w",
                MatrixToRowMajorJson(outputs.body_ang_vel_w)},
           }},
      };

      const std::string payload_string = payload.dump();
      mcap::Message message;
      message.channelId =
          invocation == "reference" ? reference_channel_.id : action_channel_.id;
      message.sequence =
          invocation == "reference" ? reference_sequence_++ : action_sequence_++;
      message.logTime = now_ns;
      message.publishTime = now_ns;
      message.dataSize = payload_string.size();
      message.data =
          reinterpret_cast<const std::byte *>(payload_string.data());

      auto status = writer_.write(message);
      if (!status.ok()) {
        DisableAfterError("write", status.message);
      }
    } catch (const std::exception &error) {
      DisableAfterError("record", error.what());
    }
  }

  void RecordJointCommandFeedback(const PolicyIoInvocationContext &context,
                                  const PolicyIoJointCommand &command) {
    if (!enabled_ || !open_) {
      return;
    }

    try {
      const uint64_t now_ns = NowNs();
      nlohmann::json payload = {
          {"schema_version", 1},
          {"runner", config_.runner},
          {"param_tag", config_.param_tag},
          {"policy_file", config_.policy_file},
          {"invocation", std::string(kJointCommandFeedbackInvocation)},
          {"iter", context.iter},
          {"runner_time_step", context.runner_time_step},
          {"runner_period_s", context.runner_period_s},
          {"log_time_ns", now_ns},
          {"topic", config_.joint_command_feedback_topic},
          {"joint_command_feedback",
           {
               {"position", VectorToJson(command.position)},
               {"velocity", VectorToJson(command.velocity)},
               {"feed_forward_torque",
                VectorToJson(command.feed_forward_torque)},
               {"torque", VectorToJson(command.torque)},
               {"stiffness", VectorToJson(command.stiffness)},
               {"damping", VectorToJson(command.damping)},
           }},
      };

      const std::string payload_string = payload.dump();
      mcap::Message message;
      message.channelId = joint_command_feedback_channel_.id;
      message.sequence = joint_command_feedback_sequence_++;
      message.logTime = now_ns;
      message.publishTime = now_ns;
      message.dataSize = payload_string.size();
      message.data =
          reinterpret_cast<const std::byte *>(payload_string.data());

      auto status = writer_.write(message);
      if (!status.ok()) {
        DisableAfterError("write joint command feedback", status.message);
      }
    } catch (const std::exception &error) {
      DisableAfterError("record joint command feedback", error.what());
    }
  }

  bool IsEnabled() const { return enabled_ && open_; }

  const std::filesystem::path &FilePath() const { return file_path_; }

private:
  std::string BuildFileName() const {
    std::ostringstream out;
    out << "rl_tracking_motion_" << SanitizeForFilename(config_.param_tag)
        << "_" << TimestampForFilename() << "_" << getpid() << ".mcap";
    return out.str();
  }

  void RegisterSchemaAndChannels() {
    const std::string schema_json = BuildSchemaJson().dump();
    schema_ = mcap::Schema(kSchemaName, kSchemaEncoding, schema_json);
    writer_.addSchema(schema_);

    reference_channel_ =
        mcap::Channel(kReferenceTopic, kMessageEncoding, schema_.id,
                      {{"invocation", "reference"}});
    writer_.addChannel(reference_channel_);

    action_channel_ = mcap::Channel(kActionTopic, kMessageEncoding, schema_.id,
                                    {{"invocation", "action"}});
    writer_.addChannel(action_channel_);

    joint_command_feedback_channel_ = mcap::Channel(
        config_.joint_command_feedback_topic, kMessageEncoding, schema_.id,
        {{"invocation", std::string(kJointCommandFeedbackInvocation)}});
    writer_.addChannel(joint_command_feedback_channel_);
  }

  void WriteMetadata() {
    mcap::Metadata metadata;
    metadata.name = std::string(kMetadataName);
    metadata.metadata["schema_version"] = "1";
    metadata.metadata["runner"] = config_.runner;
    metadata.metadata["param_tag"] = config_.param_tag;
    metadata.metadata["policy_file"] = config_.policy_file;
    metadata.metadata["joint_names"] =
        nlohmann::json(config_.joint_names).dump();
    metadata.metadata["command_joint_names"] =
        nlohmann::json(config_.command_joint_names).dump();
    metadata.metadata["observation_names"] =
        nlohmann::json(config_.observation_names).dump();
    metadata.metadata["observation_history_lengths"] =
        nlohmann::json(config_.observation_history_lengths).dump();
    metadata.metadata["observation_layout"] =
        ObservationLayoutJson(config_.observation_layout).dump();
    metadata.metadata["output_shapes"] =
        OutputShapesJson(config_.num_actions).dump();
    metadata.metadata["joint_command_feedback_topic"] =
        config_.joint_command_feedback_topic;
    metadata.metadata["joint_command_feedback_shapes"] =
        JointCommandShapesJson(
            static_cast<int>(config_.command_joint_names.size()))
            .dump();

    auto status = writer_.write(metadata);
    if (!status.ok()) {
      DisableAfterError("metadata", status.message);
      return;
    }
    metadata_written_ = true;
  }

  void DisableAfterError(std::string_view operation, std::string_view message) {
    LOG(ERROR) << "[PolicyIoMcapLogger] Disabling MCAP policy I/O logging after "
               << operation << " failure: " << message;
    if (open_) {
      writer_.terminate();
    }
    open_ = false;
    enabled_ = false;
  }

  PolicyIoMcapLoggerConfig config_;
  std::filesystem::path file_path_;
  mcap::McapWriter writer_;
  mcap::Schema schema_;
  mcap::Channel reference_channel_;
  mcap::Channel action_channel_;
  mcap::Channel joint_command_feedback_channel_;
  uint32_t reference_sequence_ = 0;
  uint32_t action_sequence_ = 0;
  uint32_t joint_command_feedback_sequence_ = 0;
  bool metadata_written_ = false;
  bool enabled_ = false;
  bool open_ = false;
};

PolicyIoMcapLogger::PolicyIoMcapLogger()
    : impl_(std::make_unique<Impl>()) {}

PolicyIoMcapLogger::~PolicyIoMcapLogger() { Close(); }

void PolicyIoMcapLogger::Start(const PolicyIoMcapLoggerConfig &config) {
  impl_->Start(config);
}

void PolicyIoMcapLogger::Close() { impl_->Close(); }

void PolicyIoMcapLogger::RecordReference(
    const PolicyIoInvocationContext &context, const Eigen::VectorXf &obs,
    int time_step, const PolicyOutputs &outputs) {
  impl_->Record("reference", context, obs, time_step, outputs);
}

void PolicyIoMcapLogger::RecordAction(
    const PolicyIoInvocationContext &context, const Eigen::VectorXf &obs,
    int time_step, const PolicyOutputs &outputs) {
  impl_->Record("action", context, obs, time_step, outputs);
}

void PolicyIoMcapLogger::RecordJointCommandFeedback(
    const PolicyIoInvocationContext &context,
    const PolicyIoJointCommand &command) {
  impl_->RecordJointCommandFeedback(context, command);
}

bool PolicyIoMcapLogger::IsEnabled() const { return impl_->IsEnabled(); }

const std::filesystem::path &PolicyIoMcapLogger::FilePath() const {
  return impl_->FilePath();
}

} // namespace runner
