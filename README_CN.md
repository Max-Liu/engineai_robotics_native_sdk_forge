# EngineAI Robotics Native SDK Fork

中文 | [English](README.md)

本仓库 fork 自官方项目
[engineai-robotics/engineai_robotics_native_sdk](https://github.com/engineai-robotics/engineai_robotics_native_sdk)。

本仓库保留了原始 EngineAI Native SDK 的架构与工具链，并在此基础上针对本地开发、部署、手柄控制和动作扩展流程做了一些实用改进。

如发现问题或有改进建议，欢迎随时提交 PR。

## 本 Fork 的主要改动

- **部署问题修复：** 修复和调整了一些真实开发、部署、运行过程中遇到的问题。
- **macOS 手柄支持：** 支持将连接到 macOS 的实体手柄输入桥接到 Docker 内的 SDK 运行时，并为 DUALSHOCK 4 等手柄提供 HID backend 支持。
- **简化动作扩展：** 当新的全身 RL tracking 动作符合共享 MNN tracking 接口时，只需添加配置和策略文件，不需要复制 runner 代码或新增每个动作专属的 C++ 类。
- **改进 macOS 仿真流程：** 在 macOS + Docker Desktop 场景下，可通过浏览器 VNC 查看 MuJoCo 仿真画面。

原始上游项目、文档与基线实现请参考：

- 上游仓库：<https://github.com/engineai-robotics/engineai_robotics_native_sdk>
- EngineAI Native SDK 文档：<https://dx3a2bminsq.feishu.cn/wiki/KyD9wDc4mi03uXkTVuAc5LQan4C>

## 概述

EngineAI Native SDK 是一个基于 C++20 的人形机器人控制框架，面向机器人应用开发、仿真验证和真机部署。它提供模块化运行时、基于 runner 的运动控制插件、机型配置、模型与参数管理、MuJoCo 仿真支持以及部署脚本。

主要模块包括：

- **运行时执行器：** 启动控制框架并管理任务执行。
- **Runner 插件：** 实现运动/控制行为，并加载策略或参数。
- **硬件驱动：** 将运行时连接到机器人硬件接口。
- **ROS2 桥接节点：** 将部分 SDK 数据和控制接口暴露给 ROS2。
- **协议定义：** 定义系统使用的消息和服务。
- **机器人资源：** 存放机器人配置、策略路径、模型和资源文件。
- **仿真工具：** 构建和运行 MuJoCo 仿真环境。
- **虚拟手柄工具：** 提供 GUI、键盘和实体手柄输入路径。

## 仓库结构

```text
native_sdk/
├── assets/
│   ├── config/              # 机器人运行时配置
│   └── resource/            # 机器人模型与资源
├── core/                    # 核心框架库
├── docker/                  # Docker 开发环境
├── docs/                    # README 图片和辅助文档
├── scripts/                 # 构建、仿真和运行辅助脚本
├── simulation/mujoco/       # MuJoCo 仿真代码
├── src/
│   ├── executor/            # 运行时入口和执行器逻辑
│   ├── runner/              # 运动/控制 runner 插件
│   ├── hardware/            # 硬件驱动
│   ├── ros2_node/           # ROS2 桥接节点
│   ├── protocol/            # 接口协议定义
│   └── data/                # 数据与参数模块
├── tools/virtual_gamepad/   # 虚拟手柄与实体手柄桥接
├── build.sh                 # SDK 主构建脚本
├── run.sh                   # 本地运行脚本
└── install.sh               # 真机部署脚本
```

生成的构建产物应放在 `build/` 中。

## 快速开始

所有编译、测试和运行命令都应在 Docker 开发环境中执行。

### 1. 安装 Docker

安装 Docker 和 Docker Compose：

```bash
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```

中国大陆用户可使用镜像源：

```bash
export DOWNLOAD_URL="https://mirrors.tuna.tsinghua.edu.cn/docker-ce"
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```

允许当前用户免 `sudo` 运行 Docker：

```bash
sudo usermod -aG docker $USER
```

执行后需要注销并重新登录，或重启系统，使 Docker 用户组变更生效。

验证安装：

```bash
docker --version
docker compose version
```

### 2. 生成开发容器

```bash
cd native_sdk
./docker/generate.sh
```

该脚本会创建容器并将当前仓库挂载到容器内，同时生成 `engineai_robotics_env` 快捷入口。

![容器开发环境生成流程](docs/generate_docker.png)

打开新的终端并进入容器：

```bash
engineai_robotics_env
```

![进入容器开发环境](docs/inside_docker.png)

### 3. 编译

```bash
engineai_robotics_env
./build.sh
```

启用 GoogleTest 目标进行 debug 构建：

```bash
./build.sh -t debug -T
```

完整构建后，单独重编某个 runner：

```bash
./build.sh -m rl_walking_example
```

### 4. 运行

```bash
engineai_robotics_env

# 使用默认机器人模型运行
./run.sh

# 指定机器人模型运行
./run.sh pm01_edu
```

启动后，运行时默认进入 `idle` 状态。

## 测试

测试仅在启用 `BUILD_TESTS` 时构建，通常通过 `./build.sh -T` 启用。

```bash
engineai_robotics_env
./build.sh -t debug -T
ctest --test-dir build --output-on-failure
```

除非明确记录硬件要求和操作员假设，否则避免编写真机硬件依赖测试。

## 手柄与遥控操作

SDK 通过有限状态机切换机器人运动状态。每个状态都声明了明确的进入条件和允许的状态转移路径，不满足条件的危险切换会被拒绝。

### Logitech F710

使用 Logitech Wireless Gamepad F710，并切换到 Xbox 模式。插入 USB 接收器后，容器环境通常会自动识别该手柄。

### macOS 实体手柄桥接

本 fork 支持将连接到 macOS 的实体手柄输入通过虚拟手柄工具转发到 Docker 运行时。

列出已连接手柄：

```bash
python3 tools/virtual_gamepad/virtual_gamepad.py --list-gamepads
```

无 GUI 运行手柄桥接：

```bash
python3 tools/virtual_gamepad/virtual_gamepad.py --no-gui \
  --lcm-url 'udpm://239.255.76.67:7667?ttl=1'
```

对于 Bluetooth DUALSHOCK 4 或其他无法被 pygame/SDL 正确识别的手柄，可使用 HID backend：

```bash
python3 tools/virtual_gamepad/virtual_gamepad.py --no-gui --backend hid \
  --lcm-url 'udpm://239.255.76.67:7667?ttl=1'
```

如果 Docker Desktop 阻断了主机到容器的 multicast 或 UDP 通信，可使用 docker-exec relay：

```bash
python3 tools/virtual_gamepad/virtual_gamepad.py --no-gui --backend hid \
  --docker-container engineai_robotics_env
```

如果 HID backend 报错 `Failed to open DUALSHOCK 4 HID input device`，请在 macOS 系统设置中为 Terminal 或当前 shell 应用授予 Input Monitoring 权限，然后重新连接手柄。

完整 CLI 选项和手柄校准说明请参考 [tools/virtual_gamepad/README.md](tools/virtual_gamepad/README.md)。

### 虚拟手柄 UI

虚拟手柄提供图形界面，支持通过键盘按键和滑条模拟手柄输入。它与 Xbox 模式下的 F710 使用相同的控制映射。

```bash
python3 tools/virtual_gamepad/virtual_gamepad.py
```

![虚拟手柄界面](docs/virtual_gamepad.png)

## 状态切换

运行时从 `idle` 状态启动。不同机器人型号的状态切换逻辑可能不同，task motion YAML 是可用状态、允许切换、runner 绑定、周期和按键绑定的准确信息来源。

主要安全回退机制是 `LB + RB`，可从任意状态强制回到 `passive`。

### pm01_edu

**状态机配置：** `assets/config/pm01_edu/task_motion/default.yaml`

| 状态 | 说明 |
|:----:|:-----|
| idle | 上电后的初始安全状态，不激活主动运动控制。 |
| passive | 阻尼状态，控制器施加被动阻尼力矩，机器人可被手动移动。 |
| pd_stand | 稳定站立控制任务，通过 PD 控制维持固定直立姿态。 |
| walk | 行走任务，机器人执行步态运动。 |
| rl_lab | RL Lab 任务，是 EngineAI 开源 RL 训练框架 [engineai_amp](https://github.com/engineai-robotics/engineai_amp) 的部署配套，用于在真实硬件上运行 engineai_amp 训练得到的策略。 |
| dance | 跳舞任务，机器人执行预设编排动作序列。 |
| stamp | 使用 `rl_tracking_motion_runner` 的全身 RL tracking 动作。 |
| power_shot | 使用 `rl_tracking_motion_runner` 的全身 RL tracking 动作。 |
| kick | 使用 `rl_tracking_motion_runner` 的全身 RL tracking 动作。 |
| jumping | 使用 `rl_tracking_motion_runner` 的全身 RL tracking 动作。 |
| bilidance | 使用 `rl_tracking_motion_runner` 的全身 RL tracking 动作。 |

当前 `pm01_edu` 按键绑定：

| 目标动作 | 触发按键 | 说明 |
|:--------:|:--------:|:-----|
| idle | LB + START | 从 `passive` 回到未激活状态。 |
| passive | LB + RB | 进入阻尼/被动状态，也作为全局安全回退。 |
| pd_stand | LB + A | 从 `passive` 进入稳定站立，或从运动状态回到站立。 |
| walk | LB + B | 进入行走。 |
| rl_lab | LB + X | 进入 RL Lab policy runner。 |
| dance | RB + B | 进入跳舞。 |
| stamp | RB + A | 触发 Stamp tracking 动作。 |
| power_shot | RB + X | 触发 Power Shot tracking 动作。 |
| kick | LB + A | 按当前 fork 配置触发 Kick tracking 动作。 |
| jumping | LB + Y | 触发 Jumping tracking 动作。 |
| bilidance | RB + Y | 触发 Bilidance tracking 动作。 |

### t800

**状态机配置：** `assets/config/t800/task_motion/default.yaml`

| 状态 | 说明 |
|:----:|:-----|
| idle | 上电后的初始安全状态，不激活主动运动控制。 |
| passive | 阻尼状态，控制器施加被动阻尼力矩，机器人可被手动移动。 |
| pd_stand | 稳定站立控制任务，通过 PD 控制维持固定直立姿态。 |
| walk | 行走任务，机器人执行步态运动。 |
| dance | 跳舞任务，机器人执行预设编排动作序列。 |

当前 `t800` 按键绑定：

| 目标动作 | 触发按键 | 说明 |
|:--------:|:--------:|:-----|
| idle | LB + START | 从 `passive` 回到未激活状态。 |
| passive | LB + RB | 进入阻尼/被动状态，也作为全局安全回退。 |
| pd_stand | LB + A | 进入稳定站立或回到站立。 |
| walk | LB + B | 进入行走。 |
| dance | RB + B | 进入跳舞。 |

### 全局安全机制

任意状态均可通过 `LB + RB` 强制切换到 `passive`。

该功能类似软急停：

- 立即终止当前运动控制逻辑。
- 将系统退回到安全的被动状态。
- 在调试和真机运行中降低运动控制失控风险。

## 新增全身 RL Tracking 动作

当策略使用共享 MNN tracking 接口时，本 fork 显著降低了新增全身 RL tracking 动作的成本。

这类动作应使用公共的 `rl_tracking_motion_runner`。不要复制现有 tracking runner，也不要为每个动作新增专属 C++ runner 或参数类，除非该策略需要不同的 tensor 接口、观测布局或无法通过 YAML 表达的 action 后处理逻辑。

### 1. 添加动作配置

创建：

```text
assets/config/<robot>/<motion_tag>/default.yaml
```

将 `policy_file` 指向该动作目录下的策略文件，通常为：

```yaml
policy_file: <motion_tag>/policies/policy.mnn
```

使用共享 tracking schema：

```yaml
time_step_total: 0
joint_names: []
joint_stiffness: []
joint_damping: []
default_joint_pos: []
action_scale: []
observation_names: []
observation_history_lengths: []
transition_duration_s: 0.0
loop_motion: false
reset_observation_history_on_loop: true
auto_transition: true
align_reference_to_robot_anchor: true
```

`observation_names` 保持以下支持顺序：

```yaml
observation_names:
  - command
  - motion_anchor_pos_b
  - motion_anchor_ori_b
  - base_ang_vel
  - joint_pos
  - joint_vel
  - actions
  - projected_gravity
  - joint_error
  - motion_phase
```

### 2. 注册参数作用域

在以下文件中添加新的 `param_tag` 和 scope：

```text
assets/config/<robot>/mode.yaml
```

示例：

```yaml
<motion_tag>: <motion_tag>/default
```

### 3. 添加任务动作入口

在以下文件中添加 motion entry：

```text
assets/config/<robot>/task_motion/default.yaml
```

使用：

```yaml
runner: rl_tracking_motion_runner
param_tag: <motion_tag>
```

在同一文件中配置 transitions、period 和 key bindings。

### 4. 构建共享 Runner

选择性构建时使用：

```bash
ENGINEAI_ROBOTICS_USED_RUNNERS=rl_tracking_motion_runner ./build.sh
```

完整构建后，可单独重编 runner：

```bash
./build.sh -m rl_tracking_motion
```

只有当动作无法使用共享 tracking YAML schema 时，才新增 C++ runner 模块。

## MuJoCo 仿真

### 编译

```bash
engineai_robotics_env
./scripts/build_mujoco.sh
```

如果 GitHub 依赖不可访问，可使用镜像模式：

```bash
./scripts/build_mujoco.sh --mirror-deps
```

### 运行

运行前请确保以下文件中的 `active_mode` 设置为 `sim`：

```text
assets/config/<robot>/mode.yaml
```

启动仿真：

```bash
engineai_robotics_env

# 默认机器人
./scripts/run_mujoco.sh

# 指定机器人
./scripts/run_mujoco.sh pm01_edu
```

启动后可通过手柄切换状态。

### 在 macOS 上通过 VNC 运行 MuJoCo

macOS + Docker Desktop 不支持直接从容器显示原生 GUI。可使用内置 VNC 流程运行 headless MuJoCo，并通过浏览器查看。

```bash
./scripts/start_mujoco_vnc.sh
```

该脚本会在需要时构建 MuJoCo，在容器中启动虚拟 framebuffer 和 x11vnc，在 macOS 上启动 noVNC WebSocket proxy，并打开：

```text
http://localhost:6080/vnc.html
```

停止 VNC 会话：

```bash
./scripts/start_mujoco_vnc.sh --stop
```

macOS 依赖：

```bash
pip3 install websockets
```

### Linux GPU 渲染

如果主机有受支持的 NVIDIA GPU 和驱动，可安装 NVIDIA Container Toolkit，在 `docker/generate.sh` 中设置 `NVIDIA_GPU_AVAILABLE=y`，并重新生成容器：

```bash
./docker/generate.sh
```

## 真机部署

真机运行属于部署敏感操作。运行运动控制前，请确认硬件状态、网络连接、操作员准备情况以及急停可用性。

### 配置部署目标

编辑 `install.sh`：

```bash
remote_user="user"
remote_host="192.168.0.163"
remote_dir="~/projects/engineai_robotics"
```

执行部署：

```bash
cd native_sdk

# ./install.sh <robot_model> <mode>
./install.sh pm01_edu robot
```

### 在机器人上运行

运行前检查：

1. 使用急停遥控器使能机器人电机系统。
2. 连接机器人热点或使用网线连接。
3. 确保人员远离机器人工作空间。
4. 首次站立和行走测试建议先将机器人悬挂在保护架上。

启动：

```bash
ssh user@192.168.0.163
sudo systemctl stop robotics.service

cd ~/projects/engineai_robotics
sudo ./run_robot.sh pm01_edu
```

后台运行：

```bash
nohup sudo ./run_robot.sh pm01_edu > nohup.out 2>&1 &
tail -f nohup.out
```

如果机器人行为异常，请立即按下物理急停，或通过 `LB + RB` 切换到 `passive`。

## 开发说明

- 使用 C++20、两个空格缩进，以及 `.cc` / `.h` 文件扩展名。
- 优先使用已有 CMake helper，例如 `get_namespace` 和 `add_googletest`。
- 尽量将机器人特定行为放在 `assets/config/<robot>/` 中。
- 对共享接口 RL tracking 动作使用 `rl_tracking_motion_runner`。
- 不要提交生成的构建产物、本地密钥、crash dump 或机器相关权限配置。

## 许可证

本项目沿用上游许可证，采用 [BSD 3-Clause License](LICENSE.txt)。
