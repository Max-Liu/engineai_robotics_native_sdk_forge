#include "lcm_interface/lcm_subscriber.h"

#include <atomic>
#include <cstddef>

namespace {
Eigen::Quaterniond NormalizedQuaternionOrIdentity(const double* quaternion) {
  Eigen::Quaterniond value(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
  if (value.norm() <= 0.0) {
    return Eigen::Quaterniond::Identity();
  }
  return value.normalized();
}
}  // namespace

LcmSubscriber::LcmSubscriber(const data::LcmParam& param, const std::shared_ptr<LcmDataStore>& lcm_data_store)
    : lcm_interface::PeriodicTask("lcm_subscriber", /* period */ 0.002), lcm_data_store_(lcm_data_store) {
  std::string url;
  int ttl = param.multicast ? param.ttl : 0;
  url = "udpm://" + param.ip_port + "?ttl=" + std::to_string(ttl);
  LOG(INFO) << "LCM URL: " << url;
  lcm_ = std::make_shared<lcm::LCM>(url);
  if (!lcm_->good()) {
    LOG(FATAL) << "LCM is not good.";
  }

  lcm_->subscribe(command_channel_, &LcmSubscriber::HandleSimCommand, this);
  lcm_->subscribe(replay_state_channel_, &LcmSubscriber::HandleSimReplayState, this);
}

void LcmSubscriber::TaskInit() {}

void LcmSubscriber::TaskRun() { lcm_->handle(); }

void LcmSubscriber::HandleSimCommand(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                                     const data::SimCommand* msg) {
  sim_command_.q = Eigen::Map<const Eigen::VectorXd>(msg->joint_position.data(), msg->num_ranges);
  sim_command_.qd = Eigen::Map<const Eigen::VectorXd>(msg->joint_velocity.data(), msg->num_ranges);
  sim_command_.tau_ff = Eigen::Map<const Eigen::VectorXd>(msg->joint_feed_forward_torque.data(), msg->num_ranges);
  sim_command_.kp = Eigen::Map<const Eigen::VectorXd>(msg->joint_stiffness.data(), msg->num_ranges);
  sim_command_.kd = Eigen::Map<const Eigen::VectorXd>(msg->joint_damping.data(), msg->num_ranges);

  lcm_data_store_->sim_command.Set(sim_command_);
}

void LcmSubscriber::HandleSimReplayState(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                                         const data::SimState* msg) {
  if (msg->num_ranges != lcm_data_store_->num_joints) {
    LOG(ERROR) << "Ignoring replay state with " << msg->num_ranges << " joints on " << channel
               << ", expected " << lcm_data_store_->num_joints;
    return;
  }

  const auto num_ranges = static_cast<size_t>(msg->num_ranges);
  if (msg->joint_position.size() != num_ranges || msg->joint_velocity.size() != num_ranges ||
      msg->joint_torque.size() != num_ranges) {
    LOG(ERROR) << "Ignoring replay state with mismatched joint vector sizes on " << channel;
    return;
  }

  sim_replay_state_.q = Eigen::Map<const Eigen::VectorXd>(msg->joint_position.data(), msg->num_ranges);
  sim_replay_state_.qd = Eigen::Map<const Eigen::VectorXd>(msg->joint_velocity.data(), msg->num_ranges);
  sim_replay_state_.tau = Eigen::Map<const Eigen::VectorXd>(msg->joint_torque.data(), msg->num_ranges);

  sim_replay_state_.base_link_position = Eigen::Map<const Eigen::Vector3d>(msg->base_link_position);
  sim_replay_state_.base_link_linear_velocity =
      Eigen::Map<const Eigen::Vector3d>(msg->base_link_linear_velocity);
  sim_replay_state_.base_link_quaternion = NormalizedQuaternionOrIdentity(msg->base_link_quaternion);
  sim_replay_state_.base_link_angular_velocity =
      Eigen::Map<const Eigen::Vector3d>(msg->base_link_angular_velocity);

  sim_replay_state_.imu_link_position = Eigen::Map<const Eigen::Vector3d>(msg->imu_link_position);
  sim_replay_state_.imu_link_linear_velocity = Eigen::Map<const Eigen::Vector3d>(msg->imu_link_linear_velocity);
  sim_replay_state_.imu_link_quaternion = NormalizedQuaternionOrIdentity(msg->imu_link_quaternion);
  sim_replay_state_.imu_link_angular_velocity = Eigen::Map<const Eigen::Vector3d>(msg->imu_link_angular_velocity);

  sim_replay_state_.imu_sensor_quaternion = NormalizedQuaternionOrIdentity(msg->imu_sensor_quaternion);
  sim_replay_state_.imu_sensor_linear_acceleration =
      Eigen::Map<const Eigen::Vector3d>(msg->imu_sensor_linear_acceleration);
  sim_replay_state_.imu_sensor_angular_velocity = Eigen::Map<const Eigen::Vector3d>(msg->imu_sensor_angular_velocity);

  sim_replay_state_.contact_force.clear();
  const int contact_dimension = lcm_data_store_->num_single_contact_dimensions;
  if (msg->num_contact_ranges > 0 && contact_dimension > 0 &&
      msg->contact_force.size() == static_cast<size_t>(msg->num_contact_ranges)) {
    const int num_contacts = msg->num_contact_ranges / contact_dimension;
    sim_replay_state_.contact_force.reserve(num_contacts);
    for (int i = 0; i < num_contacts; ++i) {
      sim_replay_state_.contact_force.emplace_back(
          Eigen::Map<const Eigen::VectorXd>(msg->contact_force.data() + i * contact_dimension, contact_dimension));
    }
  }

  lcm_data_store_->sim_replay_state.Set(sim_replay_state_);
  lcm_data_store_->sim_replay_state_sequence.fetch_add(1, std::memory_order_release);
}
