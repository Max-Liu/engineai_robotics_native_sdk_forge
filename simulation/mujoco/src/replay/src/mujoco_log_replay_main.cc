#include "mujoco_log_replay/mujoco_log_replay.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <lcm/lcm-cpp.hpp>

namespace {

struct CliOptions {
  std::filesystem::path log_path;
  std::string robot;
  std::string lcm_url;
  mujoco_log_replay::ReplayMode mode =
      mujoco_log_replay::ReplayMode::kCommand;
  std::string topic;
  double speed = 1.0;
  double hold_first_s = 2.0;
  int64_t start_iter = 0;
  int64_t end_iter = -1;
  bool dry_run = false;
  bool loop = false;
  bool topic_set = false;
};

std::atomic_bool g_stop_requested{false};

void HandleSignal(int) { g_stop_requested.store(true); }

[[noreturn]] void ThrowUsage(const std::string& message) {
  throw std::runtime_error(message + "\nRun with --help for usage.");
}

void PrintUsage(std::ostream& out) {
  out << "Usage: ./mujoco_log_replay --log <file.mcap> --robot <robot> "
         "[options]\n\n"
      << "Options:\n"
      << "  --dry-run                 Parse and validate without publishing LCM\n"
      << "  --loop                    Repeat the selected command range\n"
      << "  --lcm-url <url>           LCM URL, default from robot lcm/default.yaml\n"
      << "  --mode <command|state>    Replay mode, default command\n"
      << "  --topic <name>            LCM topic, default sim_command for command "
         "mode or sim_replay_state for state mode\n"
      << "  --speed <factor>          Replay speed multiplier, default 1.0\n"
      << "  --hold-first-s <seconds>  Publish first frame before replay, "
         "default 2.0\n"
      << "  --start-iter <iter>       First iter to include, default 0\n"
      << "  --end-iter <iter>         Last iter to include, default -1 for all\n"
      << "  --help                    Show this help text\n";
}

std::string RequiredValue(int argc, char** argv, int* index) {
  if (*index + 1 >= argc) {
    ThrowUsage(std::string("missing value for ") + argv[*index]);
  }
  ++(*index);
  return argv[*index];
}

double ParseDouble(const std::string& value, const std::string& flag) {
  try {
    size_t parsed = 0;
    double result = std::stod(value, &parsed);
    if (parsed != value.size()) {
      ThrowUsage("invalid numeric value for " + flag + ": " + value);
    }
    return result;
  } catch (const std::exception&) {
    ThrowUsage("invalid numeric value for " + flag + ": " + value);
  }
}

int64_t ParseInt64(const std::string& value, const std::string& flag) {
  try {
    size_t parsed = 0;
    int64_t result = std::stoll(value, &parsed);
    if (parsed != value.size()) {
      ThrowUsage("invalid integer value for " + flag + ": " + value);
    }
    return result;
  } catch (const std::exception&) {
    ThrowUsage("invalid integer value for " + flag + ": " + value);
  }
}

CliOptions ParseArgs(int argc, char** argv) {
  CliOptions options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      PrintUsage(std::cout);
      std::exit(0);
    } else if (arg == "--log") {
      options.log_path = RequiredValue(argc, argv, &i);
    } else if (arg == "--robot") {
      options.robot = RequiredValue(argc, argv, &i);
    } else if (arg == "--dry-run") {
      options.dry_run = true;
    } else if (arg == "--loop") {
      options.loop = true;
    } else if (arg == "--lcm-url") {
      options.lcm_url = RequiredValue(argc, argv, &i);
    } else if (arg == "--mode") {
      options.mode =
          mujoco_log_replay::ParseReplayMode(RequiredValue(argc, argv, &i));
    } else if (arg == "--topic") {
      options.topic = RequiredValue(argc, argv, &i);
      options.topic_set = true;
    } else if (arg == "--speed") {
      options.speed = ParseDouble(RequiredValue(argc, argv, &i), arg);
    } else if (arg == "--hold-first-s") {
      options.hold_first_s = ParseDouble(RequiredValue(argc, argv, &i), arg);
    } else if (arg == "--start-iter") {
      options.start_iter = ParseInt64(RequiredValue(argc, argv, &i), arg);
    } else if (arg == "--end-iter") {
      options.end_iter = ParseInt64(RequiredValue(argc, argv, &i), arg);
    } else {
      ThrowUsage("unknown argument: " + arg);
    }
  }

  if (options.log_path.empty()) {
    ThrowUsage("--log is required");
  }
  if (options.robot.empty()) {
    ThrowUsage("--robot is required");
  }
  if (!options.topic_set) {
    options.topic = mujoco_log_replay::DefaultTopicForMode(options.mode);
  }
  if (options.topic.empty()) {
    ThrowUsage("--topic must not be empty");
  }
  if (options.speed <= 0.0) {
    ThrowUsage("--speed must be greater than 0");
  }
  if (options.hold_first_s < 0.0) {
    ThrowUsage("--hold-first-s must be >= 0");
  }
  if (options.end_iter >= 0 && options.start_iter > options.end_iter) {
    ThrowUsage("--start-iter must be <= --end-iter");
  }
  return options;
}

void InterruptibleSleep(double seconds) {
  if (seconds <= 0.0) {
    return;
  }

  using Clock = std::chrono::steady_clock;
  const auto deadline = Clock::now() + std::chrono::duration<double>(seconds);
  while (!g_stop_requested.load()) {
    const auto now = Clock::now();
    if (now >= deadline) {
      return;
    }
    const auto remaining = deadline - now;
    const auto step = remaining > std::chrono::milliseconds(100)
                          ? std::chrono::milliseconds(100)
                          : remaining;
    std::this_thread::sleep_for(step);
  }
}

template <typename ReplayFrame>
double ReplayDeltaSeconds(const ReplayFrame& previous,
                          const ReplayFrame& current, double speed) {
  double delta_s = 0.0;
  if (current.log_time_ns > previous.log_time_ns) {
    delta_s = static_cast<double>(current.log_time_ns - previous.log_time_ns) *
              1e-9;
  } else if (previous.runner_period_s > 0.0) {
    delta_s = previous.runner_period_s;
  } else if (current.runner_period_s > 0.0) {
    delta_s = current.runner_period_s;
  }
  return delta_s / speed;
}

void PublishCommand(lcm::LCM* lcm, const std::string& topic,
                    const mujoco_log_replay::ReplayCommand& command) {
  data::SimCommand message = mujoco_log_replay::ToSimCommand(command);
  if (lcm->publish(topic, &message) != 0) {
    throw std::runtime_error("failed to publish LCM message on topic: " +
                             topic);
  }
}

void PublishState(lcm::LCM* lcm, const std::string& topic,
                  const mujoco_log_replay::ReplayRobotState& state) {
  data::SimState message = mujoco_log_replay::ToSimReplayState(state);
  if (lcm->publish(topic, &message) != 0) {
    throw std::runtime_error("failed to publish LCM message on topic: " +
                             topic);
  }
}

size_t HoldFirstCommand(lcm::LCM* lcm, const std::string& topic,
                        const mujoco_log_replay::ReplayCommand& command,
                        double hold_first_s) {
  if (hold_first_s <= 0.0) {
    return 0;
  }

  const double period_s =
      command.runner_period_s > 0.0 ? command.runner_period_s : 0.02;
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(hold_first_s);
  size_t published = 0;
  while (!g_stop_requested.load() &&
         std::chrono::steady_clock::now() < deadline) {
    PublishCommand(lcm, topic, command);
    ++published;
    InterruptibleSleep(period_s);
  }
  return published;
}

size_t HoldFirstState(lcm::LCM* lcm, const std::string& topic,
                      const mujoco_log_replay::ReplayRobotState& state,
                      double hold_first_s) {
  if (hold_first_s <= 0.0) {
    return 0;
  }

  const double period_s =
      state.runner_period_s > 0.0 ? state.runner_period_s : 0.02;
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(hold_first_s);
  size_t published = 0;
  while (!g_stop_requested.load() &&
         std::chrono::steady_clock::now() < deadline) {
    PublishState(lcm, topic, state);
    ++published;
    InterruptibleSleep(period_s);
  }
  return published;
}

size_t ReplayCommandsOnce(lcm::LCM* lcm, const std::string& topic,
                          const mujoco_log_replay::ReplayLog& log,
                          double speed) {
  if (log.commands.empty() || g_stop_requested.load()) {
    return 0;
  }

  size_t published = 0;
  PublishCommand(lcm, topic, log.commands.front());
  ++published;

  for (size_t i = 1; i < log.commands.size(); ++i) {
    if (g_stop_requested.load()) {
      break;
    }
    InterruptibleSleep(
        ReplayDeltaSeconds(log.commands[i - 1], log.commands[i], speed));
    if (g_stop_requested.load()) {
      break;
    }
    PublishCommand(lcm, topic, log.commands[i]);
    ++published;
  }
  return published;
}

size_t ReplayStatesOnce(lcm::LCM* lcm, const std::string& topic,
                        const mujoco_log_replay::ReplayLog& log,
                        double speed) {
  if (log.robot_states.empty() || g_stop_requested.load()) {
    return 0;
  }

  size_t published = 0;
  PublishState(lcm, topic, log.robot_states.front());
  ++published;

  for (size_t i = 1; i < log.robot_states.size(); ++i) {
    if (g_stop_requested.load()) {
      break;
    }
    InterruptibleSleep(ReplayDeltaSeconds(log.robot_states[i - 1],
                                          log.robot_states[i], speed));
    if (g_stop_requested.load()) {
      break;
    }
    PublishState(lcm, topic, log.robot_states[i]);
    ++published;
  }
  return published;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    CliOptions cli = ParseArgs(argc, argv);
    const auto repo_root = mujoco_log_replay::ResolveRepoRoot();
    cli.log_path = mujoco_log_replay::ResolveLogPath(cli.log_path, repo_root);

    mujoco_log_replay::LoadOptions load_options;
    load_options.log_path = cli.log_path;
    load_options.repo_root = repo_root;
    load_options.robot = cli.robot;
    load_options.start_iter = cli.start_iter;
    load_options.end_iter = cli.end_iter;
    mujoco_log_replay::ReplayLog log =
        mujoco_log_replay::LoadReplayLog(load_options);

    std::optional<mujoco_log_replay::RobotModelInfo> robot_model =
        mujoco_log_replay::LoadRobotModelInfo(repo_root, cli.robot, nullptr);

    if (cli.dry_run) {
      mujoco_log_replay::PrintDryRunSummary(log, robot_model, std::cout);
      return 0;
    }

    std::vector<std::string> lcm_warnings;
    if (cli.lcm_url.empty()) {
      cli.lcm_url = mujoco_log_replay::DefaultLcmUrlForRobot(
          repo_root, cli.robot, &lcm_warnings);
    }
    for (const auto& warning : log.warnings) {
      std::cerr << "[WARN] " << warning << "\n";
    }
    for (const auto& warning : lcm_warnings) {
      std::cerr << "[WARN] " << warning << "\n";
    }

    lcm::LCM lcm(cli.lcm_url);
    if (!lcm.good()) {
      throw std::runtime_error("LCM is not good for URL: " + cli.lcm_url);
    }

    const bool state_mode =
        cli.mode == mujoco_log_replay::ReplayMode::kState;
    const size_t frame_count =
        state_mode ? log.robot_states.size() : log.commands.size();

    std::cout << "Replaying " << frame_count << " "
              << mujoco_log_replay::ReplayModeName(cli.mode)
              << " frame(s) to " << cli.topic << " via " << cli.lcm_url
              << " at speed " << cli.speed << "\n";

    size_t total_published = 0;
    if (state_mode) {
      total_published += HoldFirstState(
          &lcm, cli.topic, log.robot_states.front(), cli.hold_first_s);
    } else {
      total_published += HoldFirstCommand(&lcm, cli.topic, log.commands.front(),
                                          cli.hold_first_s);
    }
    size_t cycles = 0;
    do {
      total_published += state_mode
                             ? ReplayStatesOnce(&lcm, cli.topic, log,
                                                cli.speed)
                             : ReplayCommandsOnce(&lcm, cli.topic, log,
                                                  cli.speed);
      ++cycles;
    } while (cli.loop && !g_stop_requested.load());

    std::cout << "Replay complete: published " << total_published
              << " messages across " << cycles << " cycle(s)\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << "\n";
    return 1;
  }
}
