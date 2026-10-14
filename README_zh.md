# WBC_Deploy 控制器

基于强化学习和动作跟踪的人形机器人全身控制部署系统。

[English](README.md) | 中文

## 功能特性

- **状态机控制**：包含 Passive（阻尼保护）、Loco（行走）和 WBC（全身控制）等多种 FSM 状态
- **动作跟踪**：实时跟踪重定向到 Unitree G1 人形机器人的 LAFAN1 动作数据集
- **ONNX Runtime**：使用 ONNX 模型进行快速推理
- **可配置**：基于 JSON 的配置系统，便于模式切换和参数调整

## 环境要求

- CMake >= 3.14
- C++17 编译器
- CUDA
- 依赖库：
  - unitree_sdk2
  - **ONNX Runtime 1.22.0**（见下方安装说明）
  - Eigen3
  - nlohmann_json >= 3.7.3
  - Boost

### 安装 ONNX Runtime

下载并解压 ONNX Runtime 1.22.0 到 `controller/` 目录：

**x64 平台（仿真）：**
```bash
cd controller/
wget https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-x64-1.22.0.tgz
tar -xzf onnxruntime-linux-x64-1.22.0.tgz
```

**aarch64 平台（真实机器人）：**
```bash
cd controller/
wget https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-aarch64-1.22.0.tgz
tar -xzf onnxruntime-linux-aarch64-1.22.0.tgz
```

## 编译

```bash
mkdir -p build
cd build
cmake ..
make -j4
```

## 配置

配置文件位于 `config/` 目录：
- `wbc.json`：WBC 状态配置
- `loco.json`：运动状态配置
- `fixedpose.json`：固定关节状态配置
- `passive.json`：阻尼状态配置

配置示例（`wbc.json`）：
```json
{
    "model_path": "model/wbc/lafan1_0128_1.onnx",
    "folder_path": "motion_data/lafan1/dance12_binary",
    "enter_idx": 0,
    "pause_idx": 350,
    "safe_projgravity_threshold": 0.5
}
```

## 运行

### 在 Mujoco 仿真中部署

1. 按照 https://github.com/unitreerobotics/unitree_mujoco 的说明安装 Unitree Mujoco

2. 在 `CMakeLists.txt` 中设置 ONNX Runtime 路径：
   ```cmake
   set(ONNXRUNTIME_ROOT ${PROJECT_SOURCE_DIR}/onnxruntime-linux-x64-1.22.0)
   ```

3. 使用启动参数选择通信模式，无需修改 `IOSDK.cpp`。默认是仿真，DDS domain 为 1，网卡为 `lo`。

4. 编译项目：
   ```bash
   cd build
   cmake ..
   make -j4
   ```

5. 修改unitree_mujoco/config.yaml配置，并启动仿真：
    ```yaml
    robot: "g1"  # Robot name, "go2", "b2", "b2w", "h1", "go2w", "g1"
    robot_scene: "scene_29dof.xml" # Robot scene, /unitree_robots/[robot]/scene.xml 
    domain_id: 1  # Domain id
    interface: "lo" # Interface 
    use_joystick: 1 # Simulate Unitree WirelessController using a gamepad
    joystick_type: "xbox" # support "xbox" and "switch" gamepad layout
    joystick_device: "/dev/input/js0" # Device path
    joystick_bits: 16 # Some game controllers may only have 8-bit accuracy
    print_scene_information: 1 # Print link, joint and sensors information of robot
    enable_elastic_band: 1 # Virtual spring band, used for lifting h1
    ```
   ```bash
   cd simulate/build
   ./unitree_mujoco
   ```

6. 运行控制器（在新终端中）：
   ```bash
   cd controller/build
   ./wbc_fsm --sim
   ```

### 在真实机器人上部署

1. 将本项目复制到 Unitree G1 机器人 PC2 电脑的 `/home/unitree` 目录下

2. 在 `CMakeLists.txt` 中设置 ONNX Runtime 路径：
   ```cmake
   set(ONNXRUNTIME_ROOT ${PROJECT_SOURCE_DIR}/onnxruntime-linux-aarch64-1.22.0)
   ```

3. 确认连接机器人的实际网卡名称。实机必须显式使用 `--real --interface 网卡名`，DDS domain 为 0；不接受缺省网卡或 `lo`。

4. 编译项目：
   ```bash
   cd build
   cmake ..
   make -j4
   ```

5. 运行控制器（将 `eth0` 替换为连接机器人的网卡名）：
   ```bash
   ./wbc_fsm --real --interface eth0
   ```

程序先加载所有策略和配置，再初始化通信。`handoff-v2` 移植北车框架的“先释放再等待状态”
流程：实机通过 `CheckMode` 查询原有运动服务，有活动服务时调用 `ReleaseMode`，
不再把收到有效 `rt/lowstate` 作为尝试释放的前提。RPC 客户端与 IOSDK 生命周期一致。

每次 RPC 最多等待 5 秒，重试间隔 1 秒；查询/释放的非零返回值或异常都会记录并重试。
实机接管流程总期限为 60 秒，接近期限时缩短 RPC 超时，最终仍未确认则停止启动，不发送电机命令。
成功要求连续至少 3 次查询成功且服务名为空、持续至少 2 秒；失败查询或服务重新出现会清零确认计数。
收到有效、新鲜状态后，还会在创建 `rt/lowcmd` 发布器前再次查询服务并复查状态。
释放返回成功本身不算接管成功。每次查询会打印返回值、form/name，每次释放会打印返回值。

仿真模式不创建运动服务客户端、不调用 RPC，只在 5 秒内等待有效状态。
有效状态要求 CRC、关节位置/速度、IMU 和机器人类型有效，且在最近 250 ms 内收到。
这些是启动检查，不是运行中的服务监视器或断联看门狗，也不能识别其他自定义 `rt/lowcmd` 发布程序。
异步回调和控制循环使用加锁的整帧快照；运行 WBC FSM 前应退出北车或其他底层控制程序。

部署更新后重新编译并启动，新程序会打印 `handoff-v2`、编译时间和实际可执行文件路径；
`./wbc_fsm --help` 也应显示 `handoff-v2`。仍须显式传入 `--real --interface 实际网卡`，
不带参数仍为仿真模式，不能用于实机接管。

出现 `[Startup] Ready. Starting in PASSIVE damping; no START required.` 后，起身流程为：

```text
PASSIVE → R2+X（保持阻尼）→ 松开按键，包括 R2 → R2+Y（启动起身）
```

无需先按 `START`；`START` 仍保留为可选的固定站姿功能。
接管原有运动服务后机器人将进入阻尼，阻尼不支撑站立，应在有支撑或已摆好倒地姿态时接管。
`L2+B` 返回阻尼；`SELECT`、Ctrl+C 或 SIGTERM 正常结束时会尝试下发最后一帧阻尼。
退出不会自动恢复原厂运动服务。`--help` 只显示参数，不建立 DDS 或请求接管。
实机 SDK 服务响应和闭环起身效果需单独验证，离线测试不代表实机验证。

## 项目结构

```
controller/
├── config/           # 配置文件
├── include/          # 头文件
│   ├── common/      # 通用工具
│   ├── control/     # 控制组件
│   ├── FSM/         # 状态机状态
│   ├── interface/   # 硬件接口
│   └── message/     # 消息定义
├── src/             # 源文件
│   ├── main.cpp
│   ├── control/
│   ├── FSM/
│   └── interface/
├── model/           # ONNX 模型
├── motion_data/     # 动作参考数据
└── CMakeLists.txt
```

## 控制说明

### 操作指令

- **R1**：恢复 WBC 状态（暂停时）
- **R2**：暂停 WBC 状态（在指定帧）
- **L2**：暂停 WBC 状态（在当前帧）
- **R2+A**：切换到 Loco 模式
- **L2+B**：切换到 Passive 模式
- **SELECT**：退出程序

### RL Loco+MJAmpRecovery 自动恢复任务

新增 VS Code 任务 **RL Loco+MJAmpRecovery**（“终端 → 运行任务”）。它先编译再以仿真模式启动；
也可以在仓库根目录运行：

```bash
bash scripts/run_loco_recovery.sh --sim
# 已编译时：
./build/wbc_fsm --sim --loco-auto-recovery
# 真机需要显式指定网络接口，沿用原有启动接管流程：
./build/wbc_fsm --real --interface <机器人网卡> --loco-auto-recovery
```

不加 `--loco-auto-recovery` 时保持原有 RL Loco 和独立 MJAmpRecovery 的行为。
新任务也从 PASSIVE 阻尼开始：按 `START`，等待固定站姿过渡完成，再按 `R2+B` 进入组合模式。
新任务内其他状态返回 LOCO 的路径也会进入组合模式；如果入场姿态不满足站立条件，则先运行自动恢复。

组合模式内部流程为 `LOCO → RECOVERY → RETURNING → LOCO`，不重新加载模型。
LOCO 和 RETURNING 每轮推理前读取 IMU、关节反馈：

- 倾角达到 28° 且向外倾倒速度达到 0.35 rad/s，连续 40 ms 后接管；
- 倾角达到 42° 时单次有效采样立即接管，不等待防抖；
- 向内回正及绕竖直轴转向不触发提前接管分支；
- 接管当轮直接从当前姿态初始化 Recovery 四帧历史并生成动作，自动恢复期间速度指令固定为零。

从活动 Loco 接管时，保存上一条实际发送的电机指令，并在默认 80 ms 内用平滑曲线交接
关节目标、Kp、Kd（同时交接目标速度和前馈力矩）。Recovery 从触发当轮开始推理，每帧过渡到它的最新输出。
预计达到硬倾角的时间较短时，过渡窗口会压缩；倾角达到 42°、向外倾角速度达到 2.5 rad/s，
或可用时间不足一个控制周期时，直接执行 Recovery。过渡中的新反馈也可以立即取消缓冲。
从阻尼状态进入恢复时直接建立 Recovery 支撑，不从零刚度缓慢过渡。

参数 `recovery_blend_seconds` 控制普通接管窗口（默认 0.08，允许范围 0～0.15 秒；0 关闭接管平滑），
`recovery_blend_fast_tilt_rate` 控制快速倾倒的直接接管门槛；
`recovery_blend_time_margin` 默认 0.5，用于保留预计达到硬倾角之前的响应余量。
日志会打印本次过渡时长、首帧关节目标跳变和按当前反馈估算的 PD 力矩跳变，并显示未平滑的对照值。
估算值不等于真实电机力矩，也未包含执行器限幅。人工停止、退出和异常始终优先于平滑交接。

Recovery 至少运行 1 秒；躯干倾角不超过 12°、角速度不超过 0.4 rad/s、12 个腿部关节位置
落在配置范围内且速度不超过 0.8 rad/s，连续保持 0.8 秒后，自动返回 Loco。
任何一项不满足都会清空稳定计时。该判据使用腿部姿态辅助排除坐姿/跪姿，未使用足底接触或可靠机身高度估计。
髋关节范围允许起身后分腿、脚部扭转的站姿，膝关节允许接近伸直；不要求回到默认关节姿态才返回。

返回时清空 Loco 的 LSTM、上一动作和指令缓存，并在 0.3 秒内平滑交接关节目标、Kp、Kd。
过渡期间继续检测失稳。过渡结束后，摇杆回中才重新启用运动指令，指令增益在 0.5 秒内恢复。
在 Recovery/RETURNING 中被忽略的模式按键不会在返回后因持续按住而自动执行，需松开重按。

`L2+B` 始终返回阻尼，`SELECT` 发送阻尼并退出，也会取消尚未完成的外层状态切换。
非法反馈或推理异常会退到 PASSIVE，需人工重新进入。
Recovery 超过 15 秒未满足返回条件时只打印提示，并继续运行恢复策略、保持关节支撑；
不会因为计时到期自动瘫软。每 2 秒打印当前倾角、角速度、稳定计时及阻止返回的关节位置/速度，
帮助判断为何仍在恢复阶段。`recovery_warning_seconds` 控制首次长时间恢复提示；
旧配置的 `recovery_timeout_seconds` 也按提示时限处理。
`R2+X` 仍可进入原独立恢复状态（先阻尼等待、松开按键后 `R2+Y` 启动）；
在组合模式的 LOCO 阶段，新的 `R2+A` / `R1+↑` 按键可分别切到 MJAMP / WBC。

以上阈值集中在 `config/loco_recovery.json`。站姿关节范围已按当前 Recovery 模型的隔离 MuJoCo 起身测试校准，
其他姿态、扰动力度和真机仍需验证。
模型路径仍分别读取 `config/loco.json` 和 `config/mjamp_recovery.json`。
当前控制周期为 20 ms；检测当轮生成接管指令，下一次 `sendRecv()` 发布，实际延迟还包括采样、推理和通信时间。
离线状态/FSM 测试验证控制逻辑与真实 ONNX 推理。另有可选的无 DDS MuJoCo 物理回归测试，
验证前/后向 200 N 的平滑接管、前/侧向 250 N 的直接接管，以及自动恢复、返回和方向指令行走，
每次推扰持续 0.5 秒，见 `tests/README.md`。测试读取当前配置的模型，并用相同反馈下的独立
Recovery 输出比较指令跳变；更大力度、不同起始姿态、连续恢复及真机仍需验证。

### RL Loco+MJAmpRecovery2：跌倒卸力，R1 启动起身

VS Code 任务 **RL Loco+MJAmpRecovery2** 使用独立组合状态与
`config/loco_recovery2.json`。原 **RL Loco+MJAmpRecovery** 任务和配置保持原行为。
在终端 3 停止旧控制器后，可以从任意目录运行新脚本：

```bash
bash ~/code_y7p_deploy/wpc_fsm1/wbc_fsm/scripts/run_loco_recovery2.sh --sim
# 或在仓库根目录直接运行已编译程序：
./build/wbc_fsm --sim --loco-recovery2
```

两个组合任务的启动标志互斥。新任务同样从 PASSIVE 开始，按 `START`、等待固定站姿完成，
再按 `R2+B` 进入 Loco。所有返回 LOCO 的入口都会进入本次选择的组合任务。
内部流程为 `LOCO → PROTECT_WAIT → RECOVERY → RETURNING → LOCO`：

- LOCO 或返回过渡中，倾角达到 55° 连续 40 ms，或达到 60° 单次有效采样，进入保护等待；
  此判据检测明显倾倒，不要求向外倾角速度，也不表示已检测到地面接触。
- 保护等待中停止两种策略推理，持续写入所有关节的 `Kp=0、dq=0、tau=0`。
  `protection_kd` 默认 0，因此 PD 输出为零、关节卸力；设为正值时提供速度阻尼。
  这是控制指令层的卸力，底层电机模式仍启用，不表示驱动器断电或切换电气高阻模式。
- 没有自动起身倒计时。先松开按键，再重新按**单独 R1**才启动 Recovery。
  进入保护前已按住 R1、R1+方向、方向键及其他模式键均不能启动或绕过等待。
  如果进入新任务时已经不满足站立条件，也先保护等待。
- R1 启动当轮用当前关节/IMU 反馈重新初始化 Recovery 历史，立即建立策略支撑。
  起身期间速度指令固定为零，持续按 R1 不会重复初始化；起身动作中的躺姿/跪姿不会重新触发等待。
- 站稳后沿用连续稳定判据和 0.3 秒平滑返回；摇杆回中后，运动指令在 0.5 秒内恢复。
  LOCO 或返回过渡中再次跌倒，重新卸力并等待一次新的 R1。

保护等待内部不允许模式按键绕过；LOCO 阶段仍可按 `R2+X` 进入独立手动恢复，
按 `R2+A` / `R1+↑` 切换 MJAMP / WBC。`L2+B` 始终返回原 PASSIVE 阻尼，
`SELECT` 发送阻尼并退出，非法反馈或推理异常也退到 PASSIVE。
Recovery 长时间未站稳只提示并保持策略支撑；保护等待时间不计入站稳时长。
模型继续使用 `config/loco.json` 和 `config/mjamp_recovery.json`，不复制权重文件。
卸力后的落地冲击与后续起身效果需要通过物理场景验证，离线测试不代表真机效果。

### 独立恢复策略 MJAMP_RECOVERY

项目保留两套独立的 mjlab 策略：

| 状态 | 模型 | 配置 |
| --- | --- | --- |
| `MJAMP` | `model/loco/Unitree-G1-AMP-Flat_model_30000.onnx` | `config/mjamp.json` |
| `MJAMP_RECOVERY` | 由 `config/mjamp_recovery.json` 的 `model_path` 指定 | `config/mjamp_recovery.json` |

初版恢复模型为 `model_40000.onnx`；当前部署以配置中的路径及启动打印的模型路径为准。
旧 `AMP` 和 `LOCO` 状态也继续保留。

| 当前状态 | 按键 | 行为 |
| --- | --- | --- |
| `PASSIVE`、`FIXEDSTAND`、`MJAMP`、`LOCO` | `R2+X` | 进入 `MJAMP_RECOVERY` 的 `WAITING` 阻尼等待阶段 |
| `MJAMP_RECOVERY / WAITING` | 松开按键，再按 `R2+Y` | 从此刻实际姿态启动起身策略，进入 `RUNNING` |
| `MJAMP_RECOVERY / RUNNING` | `R2+A` | 满足切换门槛后进入原有 `MJAMP` |
| `MJAMP_RECOVERY / RUNNING` | `R2+B` | 满足切换门槛后进入原有 `LOCO` |
| `MJAMP_RECOVERY` | `L2+B` | 返回阻尼状态 |
| `MJAMP_RECOVERY / RUNNING` | `R2+↑` / `R2+↓` | 切换恢复策略的快 / 慢速度范围，默认慢速 |
| `MJAMP_RECOVERY` | `SELECT` | 发送阻尼命令并退出程序 |

倒地测试时，从 `PASSIVE` 直接按 `R2+X`，无需先按 `START` 经过固定站姿插值。
此时停留在 `WAITING`，持续输出 `Kp=0、Kd=passive_kds（当前为10）、dq=0、tau=0`，不运行策略推理。
可在等待期间摆放机器人，等待多久都不会自动启动；阻尼只抵抗关节运动，不锁定姿态或支撑站立。
准备好后松开组合键（包括 R2），再按 `R2+Y`。进入时已经按住启动键或一直按住 `R2+X` 不会启动。
等待时 `R2+A/B`、快慢速切换等指令均不生效，`L2+B` 和 `SELECT` 始终可用。

启动时用此刻实际姿态重新填满四帧历史，并清零上一帧动作、速度指令和平滑缓存。
传感器、控制周期及第一帧策略输出检查通过后，才进入 `RUNNING` 并输出策略 PD 命令。
启动检查失败时保持阻尼并返回 `PASSIVE`；再次进入仍需重新松键并按 `R2+Y`。
启动后将左右摇杆回中，才启用速度指令；起身后可以继续使用恢复策略，或手动切回旧策略。
运行时再次按住或重复按 `R2+Y` 不会重置策略，也不会暂停；停止请使用 `L2+B`。
不会自动切回行走状态。此功能控制启动时机，不改变模型起身动作的速度或力度。

切回旧策略的默认门槛为启动后连续 0.5 秒倾角小于 20°、机体角速度模长小于 0.5 rad/s，等待时间不计入。
对应配置为 `switch_max_tilt_rad`、`switch_max_angular_velocity`、`switch_hold_seconds`。
门槛未满足时会提示并拒绝本次切换；请松开组合键，等待稳定后重新按下。
这些是待闭环仿真调整的初始值，仅检查姿态和角速度，不能识别坐姿或确认脚底支撑。

恢复状态默认 `enable_orientation_exit: false`，允许策略在大倾角下运行。
若启用该选项，超过 `orientation_exit_angle_rad` 会进入阻尼状态。
观测或动作非有限、IMU 四元数无效、推理异常同样触发阻尼退出。
`clip_observations` / `clip_actions` 为 `null` 表示不裁剪，与本次训练配置一致。
策略周期固定为 0.02 秒（50 Hz），使用 384 维原始观测和 29 维动作；归一化包含在 ONNX 内。

此实现已通过离线状态测试和 Python/C++ 数值对照；离线倒地观测推理不代表闭环起身成功。
起身成功率、不同初始姿态以及切回旧策略的实际稳定性需要在 MuJoCo 中验证。
离线验证命令见 [tests/README.md](tests/README.md)。

### 行走和 WBC 操作步骤（倒地起身使用上面的恢复流程）

1. 运行程序后，机器人处于**阻尼保护模式**
2. 按 **START** 键进入位控模式
3. 将机器人悬吊起来（在仿真中默认启用 `enable_elastic_band`，按键盘数字键 **9** 可以松开绑带，再次按下可重新悬吊，数字键 **8** 下放，数字键 **7** 上拉）
4. 按遥控器上的 **R2+A** 进入 Loco(AMP) Mode，此时松开吊绳
   - 按 **R2+up** 可以进入快速模式（跑步）
   - 按 **R2+down** 可以进入慢速模式（行走）
5. 按遥控器上的 **R2+A** 进入 Loco(RL) Mode
6. 按遥控器上的 **R1+Up** 进入全身控制模式（WBC Mode）
   - 按 **R2**或**L2** 可以暂停动作
   - 按 **R1** 可以继续动作

## 许可证

本项目基于 Unitree Robotics SDK2 框架开发。

原始框架：Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. 保留所有权利。

修改和扩展：[ccrpRepo / ZSTU Robotics] © 2026

## 致谢

- 基于 Unitree Robotics SDK2 开发
- 动作数据来自 LAFAN1 数据集
- 使用 ONNX Runtime 进行模型推理
