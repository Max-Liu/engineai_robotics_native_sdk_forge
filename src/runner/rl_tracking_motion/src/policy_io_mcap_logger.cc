#include "rl_tracking_motion/policy_io_mcap_logger.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cctype>
#include <cstddef>
#include <cstdlib>
#include <ctime>
#include <deque>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <utility>
#include <variant>

#include <glog/logging.h>
#include <mcap/writer.hpp>
#include <nlohmann/json.hpp>
#include <unistd.h>

namespace runner {
namespace {

constexpr int kSchemaVersion = 2;
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
constexpr std::string_view kRobotStateInvocation = "robot_state";
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

nlohmann::json VectorToJson(const Eigen::Vector3d &value) {
  return {value.x(), value.y(), value.z()};
}

nlohmann::json QuaternionWxyzToJson(const Eigen::Quaterniond &value) {
  return {value.w(), value.x(), value.y(), value.z()};
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

nlohmann::json JointStateShapesJson(int num_joints) {
  return {
      {"position", {num_joints}},
      {"velocity", {num_joints}},
      {"torque", {num_joints}},
  };
}

nlohmann::json ImuShapesJson() {
  return {
      {"quaternion_wxyz", {4}},
      {"rpy", {3}},
      {"linear_acceleration", {3}},
      {"angular_velocity", {3}},
  };
}

nlohmann::json LinkStateShapesJson() {
  return {
      {"position", {3}},
      {"quaternion_wxyz", {4}},
      {"linear_velocity", {3}},
      {"angular_velocity", {3}},
      {"linear_acceleration", {3}},
      {"angular_acceleration", {3}},
  };
}

nlohmann::json RobotStateShapesJson(int num_joints) {
  return {
      {"joint_state", JointStateShapesJson(num_joints)},
      {"motor_state", JointStateShapesJson(num_joints)},
      {"imu", ImuShapesJson()},
      {"base_state_in_world", LinkStateShapesJson()},
      {"simulated_base_state_in_world", LinkStateShapesJson()},
      {"selected_anchor_state", LinkStateShapesJson()},
  };
}

nlohmann::json JointStateJson(const PolicyIoJointState &state) {
  return {
      {"position", VectorToJson(state.position)},
      {"velocity", VectorToJson(state.velocity)},
      {"torque", VectorToJson(state.torque)},
  };
}

nlohmann::json ImuStateJson(const PolicyIoImuState &state) {
  return {
      {"quaternion_wxyz", QuaternionWxyzToJson(state.quaternion)},
      {"rpy", VectorToJson(state.rpy)},
      {"linear_acceleration", VectorToJson(state.linear_acceleration)},
      {"angular_velocity", VectorToJson(state.angular_velocity)},
  };
}

nlohmann::json LinkStateJson(const PolicyIoLinkState &state) {
  return {
      {"position", VectorToJson(state.position)},
      {"quaternion_wxyz", QuaternionWxyzToJson(state.quaternion)},
      {"linear_velocity", VectorToJson(state.linear_velocity)},
      {"angular_velocity", VectorToJson(state.angular_velocity)},
      {"linear_acceleration", VectorToJson(state.linear_acceleration)},
      {"angular_acceleration", VectorToJson(state.angular_acceleration)},
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
           {"schema_version", {{"type", "integer"}, {"const", kSchemaVersion}}},
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
           {"robot_state",
            {{"type", "object"},
             {"properties",
              {
                  {"joint_state", {{"type", "object"}}},
                  {"motor_state", {{"type", "object"}}},
                  {"imu", {{"type", "object"}}},
                  {"base_state_in_world", {{"type", "object"}}},
                  {"simulated_base_state_in_world", {{"type", "object"}}},
                  {"selected_anchor_state", {{"type", "object"}}},
                  {"selected_anchor_source", {{"type", "string"}}},
              }}}},
       }},
  };
}

} // namespace

class PolicyIoMcapLogger::Impl {
public:
  void Start(const PolicyIoMcapLoggerConfig &config) {
    Close();
    ResetRuntimeState();
    config_ = config;
    max_pending_messages_ = config_.max_pending_messages;
    file_path_.clear();

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

      open_.store(true, std::memory_order_release);
      RegisterSchemaAndChannels();
      writer_thread_ = std::thread(&Impl::WriterLoop, this);
      accepting_.store(true, std::memory_order_release);
      LOG(INFO) << "[PolicyIoMcapLogger] Writing RL tracking policy I/O to "
                << file_path_;
    } catch (const std::exception &error) {
      DisableAfterError("start", error.what());
    }
  }

  void Close() {
    accepting_.store(false, std::memory_order_release);
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      stopping_ = true;
    }
    queue_cv_.notify_one();

    if (writer_thread_.joinable()) {
      writer_thread_.join();
    }

    if (!open_.load(std::memory_order_acquire)) {
      return;
    }

    try {
      if (!metadata_written_) {
        WriteMetadata();
      }
      if (!open_.load(std::memory_order_acquire)) {
        return;
      }
      writer_.close();
    } catch (const std::exception &error) {
      LOG(ERROR) << "[PolicyIoMcapLogger] Failed to close MCAP file "
                 << file_path_ << ": " << error.what();
      writer_.terminate();
    }

    open_.store(false, std::memory_order_release);
  }

  void Record(std::string_view invocation,
              const PolicyIoInvocationContext &context,
              const Eigen::VectorXf &obs, int time_step,
              const PolicyOutputs &outputs) {
    if (!IsEnabled()) {
      DropAfterErrorIfNeeded();
      return;
    }

    PolicyRecord record;
    record.invocation =
        invocation == "reference" ? PolicyInvocation::kReference
                                  : PolicyInvocation::kAction;
    record.context = context;
    record.obs = obs;
    record.time_step = time_step;
    record.outputs = outputs;
    record.log_time_ns = NowNs();
    TryEnqueue(PendingRecord{std::move(record)});
  }

  void RecordJointCommandFeedback(const PolicyIoInvocationContext &context,
                                  const PolicyIoJointCommand &command) {
    if (!IsEnabled()) {
      DropAfterErrorIfNeeded();
      return;
    }

    JointCommandFeedbackRecord record;
    record.context = context;
    record.command = command;
    record.log_time_ns = NowNs();
    TryEnqueue(PendingRecord{std::move(record)});
  }

  void RecordRobotState(const PolicyIoInvocationContext &context,
                        const PolicyIoRobotState &state) {
    if (!IsEnabled()) {
      DropAfterErrorIfNeeded();
      return;
    }

    RobotStateRecord record;
    record.context = context;
    record.state = state;
    record.log_time_ns = NowNs();
    TryEnqueue(PendingRecord{std::move(record)});
  }

  bool IsEnabled() const {
    return accepting_.load(std::memory_order_acquire) &&
           open_.load(std::memory_order_acquire) &&
           !writer_failed_.load(std::memory_order_acquire);
  }

  PolicyIoMcapLoggerStats GetStats() const {
    PolicyIoMcapLoggerStats stats;
    stats.enqueued_messages =
        enqueued_messages_.load(std::memory_order_relaxed);
    stats.written_messages = written_messages_.load(std::memory_order_relaxed);
    stats.dropped_queue_full =
        dropped_queue_full_.load(std::memory_order_relaxed);
    stats.dropped_lock_busy =
        dropped_lock_busy_.load(std::memory_order_relaxed);
    stats.dropped_after_error =
        dropped_after_error_.load(std::memory_order_relaxed);
    stats.write_errors = write_errors_.load(std::memory_order_relaxed);
    stats.max_queue_depth = max_queue_depth_.load(std::memory_order_relaxed);
    return stats;
  }

  const std::filesystem::path &FilePath() const { return file_path_; }

private:
  enum class PolicyInvocation {
    kReference,
    kAction,
  };

  struct PolicyRecord {
    PolicyInvocation invocation = PolicyInvocation::kReference;
    PolicyIoInvocationContext context;
    Eigen::VectorXf obs;
    int time_step = 0;
    PolicyOutputs outputs;
    uint64_t log_time_ns = 0;
  };

  struct JointCommandFeedbackRecord {
    PolicyIoInvocationContext context;
    PolicyIoJointCommand command;
    uint64_t log_time_ns = 0;
  };

  struct RobotStateRecord {
    PolicyIoInvocationContext context;
    PolicyIoRobotState state;
    uint64_t log_time_ns = 0;
  };

  using PendingRecord =
      std::variant<PolicyRecord, JointCommandFeedbackRecord, RobotStateRecord>;

  std::string BuildFileName() const {
    std::ostringstream out;
    out << "rl_tracking_motion_" << SanitizeForFilename(config_.param_tag)
        << "_" << TimestampForFilename() << "_" << getpid() << ".mcap";
    return out.str();
  }

  void ResetRuntimeState() {
    reference_sequence_ = 0;
    action_sequence_ = 0;
    joint_command_feedback_sequence_ = 0;
    robot_state_sequence_ = 0;
    metadata_written_ = false;
    accepting_.store(false, std::memory_order_release);
    writer_failed_.store(false, std::memory_order_release);
    open_.store(false, std::memory_order_release);
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      pending_records_.clear();
      stopping_ = false;
    }
    enqueued_messages_.store(0, std::memory_order_relaxed);
    written_messages_.store(0, std::memory_order_relaxed);
    dropped_queue_full_.store(0, std::memory_order_relaxed);
    dropped_lock_busy_.store(0, std::memory_order_relaxed);
    dropped_after_error_.store(0, std::memory_order_relaxed);
    write_errors_.store(0, std::memory_order_relaxed);
    max_queue_depth_.store(0, std::memory_order_relaxed);
  }

  void DropAfterErrorIfNeeded() {
    if (writer_failed_.load(std::memory_order_acquire)) {
      dropped_after_error_.fetch_add(1, std::memory_order_relaxed);
    }
  }

  void TryEnqueue(PendingRecord record) {
    if (writer_failed_.load(std::memory_order_acquire)) {
      dropped_after_error_.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    if (!accepting_.load(std::memory_order_acquire) ||
        !open_.load(std::memory_order_acquire)) {
      return;
    }

    std::unique_lock<std::mutex> lock(queue_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
      dropped_lock_busy_.fetch_add(1, std::memory_order_relaxed);
      return;
    }

    if (writer_failed_.load(std::memory_order_acquire)) {
      dropped_after_error_.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    if (!accepting_.load(std::memory_order_acquire) ||
        !open_.load(std::memory_order_acquire)) {
      return;
    }
    if (pending_records_.size() >= max_pending_messages_) {
      dropped_queue_full_.fetch_add(1, std::memory_order_relaxed);
      return;
    }

    pending_records_.push_back(std::move(record));
    const auto queue_depth = pending_records_.size();
    enqueued_messages_.fetch_add(1, std::memory_order_relaxed);
    UpdateMaxQueueDepth(queue_depth);
    lock.unlock();
    queue_cv_.notify_one();
  }

  void UpdateMaxQueueDepth(std::size_t queue_depth) {
    uint64_t current = max_queue_depth_.load(std::memory_order_relaxed);
    while (queue_depth > current &&
           !max_queue_depth_.compare_exchange_weak(
               current, static_cast<uint64_t>(queue_depth),
               std::memory_order_relaxed)) {
    }
  }

  void WriterLoop() {
    std::deque<PendingRecord> batch;
    while (true) {
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [this] {
          return stopping_ || !pending_records_.empty() ||
                 writer_failed_.load(std::memory_order_acquire);
        });
        if (pending_records_.empty()) {
          if (stopping_ ||
              writer_failed_.load(std::memory_order_acquire)) {
            break;
          }
          continue;
        }
        batch.swap(pending_records_);
      }

      while (!batch.empty()) {
        if (writer_failed_.load(std::memory_order_acquire)) {
          dropped_after_error_.fetch_add(batch.size(),
                                         std::memory_order_relaxed);
          batch.clear();
          DropPendingAfterError();
          return;
        }

        std::visit([this](const auto &record) { WriteRecord(record); },
                   batch.front());
        batch.pop_front();
      }

      if (writer_failed_.load(std::memory_order_acquire)) {
        DropPendingAfterError();
        return;
      }
    }
  }

  void DropPendingAfterError() {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    const auto dropped = pending_records_.size();
    pending_records_.clear();
    stopping_ = true;
    if (dropped > 0) {
      dropped_after_error_.fetch_add(dropped, std::memory_order_relaxed);
    }
  }

  void WriteRecord(const PolicyRecord &record) {
    try {
      const bool is_reference =
          record.invocation == PolicyInvocation::kReference;
      const std::string_view invocation = is_reference ? "reference" : "action";
      nlohmann::json payload = {
          {"schema_version", kSchemaVersion},
          {"runner", config_.runner},
          {"param_tag", config_.param_tag},
          {"policy_file", config_.policy_file},
          {"invocation", std::string(invocation)},
          {"iter", record.context.iter},
          {"runner_time_step", record.context.runner_time_step},
          {"runner_period_s", record.context.runner_period_s},
          {"log_time_ns", record.log_time_ns},
          {"inputs",
           {
               {"obs", VectorToJson(record.obs)},
               {"time_step", {static_cast<float>(record.time_step)}},
           }},
          {"outputs",
           {
               {"actions", VectorToJson(record.outputs.actions)},
               {"joint_pos", VectorToJson(record.outputs.joint_pos)},
               {"joint_vel", VectorToJson(record.outputs.joint_vel)},
               {"body_pos_w",
                MatrixToRowMajorJson(record.outputs.body_pos_w)},
               {"body_quat_w",
                MatrixToRowMajorJson(record.outputs.body_quat_w)},
               {"body_lin_vel_w",
                MatrixToRowMajorJson(record.outputs.body_lin_vel_w)},
               {"body_ang_vel_w",
                MatrixToRowMajorJson(record.outputs.body_ang_vel_w)},
           }},
      };

      const std::string payload_string = payload.dump();
      mcap::Message message;
      message.channelId =
          is_reference ? reference_channel_.id : action_channel_.id;
      message.sequence =
          is_reference ? reference_sequence_++ : action_sequence_++;
      message.logTime = record.log_time_ns;
      message.publishTime = record.log_time_ns;
      message.dataSize = payload_string.size();
      message.data =
          reinterpret_cast<const std::byte *>(payload_string.data());

      auto status = writer_.write(message);
      if (!status.ok()) {
        DisableAfterError("write", status.message);
        return;
      }
      written_messages_.fetch_add(1, std::memory_order_relaxed);
    } catch (const std::exception &error) {
      DisableAfterError("record", error.what());
    }
  }

  void WriteRecord(const JointCommandFeedbackRecord &record) {
    try {
      nlohmann::json payload = {
          {"schema_version", kSchemaVersion},
          {"runner", config_.runner},
          {"param_tag", config_.param_tag},
          {"policy_file", config_.policy_file},
          {"invocation", std::string(kJointCommandFeedbackInvocation)},
          {"iter", record.context.iter},
          {"runner_time_step", record.context.runner_time_step},
          {"runner_period_s", record.context.runner_period_s},
          {"log_time_ns", record.log_time_ns},
          {"topic", config_.joint_command_feedback_topic},
          {"joint_command_feedback",
           {
               {"position", VectorToJson(record.command.position)},
               {"velocity", VectorToJson(record.command.velocity)},
               {"feed_forward_torque",
                VectorToJson(record.command.feed_forward_torque)},
               {"torque", VectorToJson(record.command.torque)},
               {"stiffness", VectorToJson(record.command.stiffness)},
               {"damping", VectorToJson(record.command.damping)},
           }},
      };

      const std::string payload_string = payload.dump();
      mcap::Message message;
      message.channelId = joint_command_feedback_channel_.id;
      message.sequence = joint_command_feedback_sequence_++;
      message.logTime = record.log_time_ns;
      message.publishTime = record.log_time_ns;
      message.dataSize = payload_string.size();
      message.data =
          reinterpret_cast<const std::byte *>(payload_string.data());

      auto status = writer_.write(message);
      if (!status.ok()) {
        DisableAfterError("write joint command feedback", status.message);
        return;
      }
      written_messages_.fetch_add(1, std::memory_order_relaxed);
    } catch (const std::exception &error) {
      DisableAfterError("record joint command feedback", error.what());
    }
  }

  void WriteRecord(const RobotStateRecord &record) {
    try {
      nlohmann::json payload = {
          {"schema_version", kSchemaVersion},
          {"runner", config_.runner},
          {"param_tag", config_.param_tag},
          {"policy_file", config_.policy_file},
          {"invocation", std::string(kRobotStateInvocation)},
          {"iter", record.context.iter},
          {"runner_time_step", record.context.runner_time_step},
          {"runner_period_s", record.context.runner_period_s},
          {"log_time_ns", record.log_time_ns},
          {"topic", config_.robot_state_topic},
          {"robot_state",
           {
               {"joint_state", JointStateJson(record.state.joint_state)},
               {"motor_state", JointStateJson(record.state.motor_state)},
               {"imu", ImuStateJson(record.state.imu)},
               {"base_state_in_world",
                LinkStateJson(record.state.base_state_in_world)},
               {"simulated_base_state_in_world",
                LinkStateJson(record.state.simulated_base_state_in_world)},
               {"selected_anchor_state",
                LinkStateJson(record.state.selected_anchor_state)},
               {"selected_anchor_source",
                record.state.selected_anchor_source},
           }},
      };

      const std::string payload_string = payload.dump();
      mcap::Message message;
      message.channelId = robot_state_channel_.id;
      message.sequence = robot_state_sequence_++;
      message.logTime = record.log_time_ns;
      message.publishTime = record.log_time_ns;
      message.dataSize = payload_string.size();
      message.data =
          reinterpret_cast<const std::byte *>(payload_string.data());

      auto status = writer_.write(message);
      if (!status.ok()) {
        DisableAfterError("write robot state", status.message);
        return;
      }
      written_messages_.fetch_add(1, std::memory_order_relaxed);
    } catch (const std::exception &error) {
      DisableAfterError("record robot state", error.what());
    }
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

    robot_state_channel_ =
        mcap::Channel(config_.robot_state_topic, kMessageEncoding, schema_.id,
                      {{"invocation", std::string(kRobotStateInvocation)}});
    writer_.addChannel(robot_state_channel_);
  }

  void WriteMetadata() {
    mcap::Metadata metadata;
    metadata.name = std::string(kMetadataName);
    metadata.metadata["schema_version"] = std::to_string(kSchemaVersion);
    metadata.metadata["runner"] = config_.runner;
    metadata.metadata["param_tag"] = config_.param_tag;
    metadata.metadata["policy_file"] = config_.policy_file;
    metadata.metadata["joint_names"] =
        nlohmann::json(config_.joint_names).dump();
    metadata.metadata["command_joint_names"] =
        nlohmann::json(config_.command_joint_names).dump();
    metadata.metadata["state_joint_names"] =
        nlohmann::json(config_.state_joint_names).dump();
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
    metadata.metadata["robot_state_topic"] = config_.robot_state_topic;
    metadata.metadata["robot_state_shapes"] =
        RobotStateShapesJson(static_cast<int>(config_.state_joint_names.size()))
            .dump();

    auto status = writer_.write(metadata);
    if (!status.ok()) {
      DisableAfterError("metadata", status.message);
      return;
    }
    metadata_written_ = true;
  }

  void DisableAfterError(std::string_view operation, std::string_view message) {
    accepting_.store(false, std::memory_order_release);
    const bool first_error =
        !writer_failed_.exchange(true, std::memory_order_acq_rel);
    if (first_error) {
      write_errors_.fetch_add(1, std::memory_order_relaxed);
      LOG(ERROR)
          << "[PolicyIoMcapLogger] Disabling MCAP policy I/O logging after "
          << operation << " failure: " << message;
    }
    if (open_.exchange(false, std::memory_order_acq_rel)) {
      writer_.terminate();
    }
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      stopping_ = true;
    }
    queue_cv_.notify_one();
  }

  PolicyIoMcapLoggerConfig config_;
  std::filesystem::path file_path_;
  mcap::McapWriter writer_;
  mcap::Schema schema_;
  mcap::Channel reference_channel_;
  mcap::Channel action_channel_;
  mcap::Channel joint_command_feedback_channel_;
  mcap::Channel robot_state_channel_;
  uint32_t reference_sequence_ = 0;
  uint32_t action_sequence_ = 0;
  uint32_t joint_command_feedback_sequence_ = 0;
  uint32_t robot_state_sequence_ = 0;
  bool metadata_written_ = false;
  std::size_t max_pending_messages_ = 4096;

  std::atomic<bool> accepting_{false};
  std::atomic<bool> writer_failed_{false};
  std::atomic<bool> open_{false};
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<PendingRecord> pending_records_;
  std::thread writer_thread_;
  bool stopping_ = false;

  std::atomic<uint64_t> enqueued_messages_{0};
  std::atomic<uint64_t> written_messages_{0};
  std::atomic<uint64_t> dropped_queue_full_{0};
  std::atomic<uint64_t> dropped_lock_busy_{0};
  std::atomic<uint64_t> dropped_after_error_{0};
  std::atomic<uint64_t> write_errors_{0};
  std::atomic<uint64_t> max_queue_depth_{0};
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

void PolicyIoMcapLogger::RecordRobotState(
    const PolicyIoInvocationContext &context, const PolicyIoRobotState &state) {
  impl_->RecordRobotState(context, state);
}

bool PolicyIoMcapLogger::IsEnabled() const { return impl_->IsEnabled(); }

PolicyIoMcapLoggerStats PolicyIoMcapLogger::GetStats() const {
  return impl_->GetStats();
}

const std::filesystem::path &PolicyIoMcapLogger::FilePath() const {
  return impl_->FilePath();
}

} // namespace runner
