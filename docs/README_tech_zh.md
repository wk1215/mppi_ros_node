# mppi_swerve_drive_ros 技术框架与二次开发指南

本文件基于当前 `mppi_swerve_drive_ros` 源码结构，对整体技术框架和二次开发要点进行整理，方便在现有导航工程中复用或改造。

---

## 1. 整体架构概览

### 1.1 系统组成

- 仿真层：Gazebo 世界 + 舵轮车模型（4WIDS），由 `world_handler`、`vel_driver`、`groundtruth_odom_publisher` 等包提供。
- 感知/定位层：
  - SLAM：`gmapping`（建图）
  - 定位：`amcl`（粒子滤波定位）
  - 地图服务：`map_server`
- 路径规划：
  - 全局规划：`move_base` + Navfn，全局路径话题 `/move_base/NavfnROS/plan`
  - 局部规划/控制：本项目提供的 **MPPI 控制器（mppi_3d / mppi_4d / mppi_h）**
- 参考代价地图：`reference_costmap_generator` 根据全局路径生成距离/姿态代价图，辅助 MPPI 评分。
- 操作与可视化：`joy_controller`（手柄操作）、`map_visualizer`（3D 地图）、`rviz` 配置等。

### 1.2 MPPI 控制器角色

在 `navigation.launch` 中，MPPI 控制器取代传统 `move_base` 的本地规划器：

1. `move_base` 仍负责 **全局路径规划** 与 **局部 costmap 维护**；
2. MPPI 控制器订阅：
   - 机器人状态 `/groundtruth_odom`（或其他里程计话题）
   - 全局路径 `/move_base/NavfnROS/plan`
   - 局部 costmap `/move_base/local_costmap/costmap`
   - 参考代价图 `/distance_error_map` 与 `/ref_yaw_map`
3. MPPI 控制器输出 `/cmd_vel`，由 `vel_driver` 转换为 8 自由度车轮关节指令驱动仿真车辆。

---

## 2. 关键包与节点

### 2.1 控制器包（src/control）

#### mppi_3d

- 作用：在 3 维控制空间 $(v_x, v_y, \omega)$ 上进行 MPPI 采样与优化。
- 典型节点：`mppi_3d_node`
- 变体：
  - `mppi_3d_a`：速度噪声较大，趋向更快、更激进的行为；
  - `mppi_3d_b`：速度噪声较小，角速度噪声相对较大，更偏安全/平稳。

### 2.2 规划与代价图（src/planning）

#### reference_costmap_generator

- 输入：
  - `/move_base/NavfnROS/plan`：全局规划路径（nav_msgs/Path）
  - `/map`：全局 OccupancyGrid 地图
- 输出：
  - `/distance_error_map`：每个栅格到参考路径最近点的距离（越远代价越高）；
  - `/ref_yaw_map`：每个栅格处的参考航向角，用于姿态对齐代价。
- 意义：为 MPPI 提供
  - “沿参考路径行驶”的奖励/惩罚；
  - “朝向与路径方向一致”的惩罚项。

### 2.3 仿真与操作（src/simulation, src/operation, src/visualization）

- `vel_driver`：订阅 `/cmd_vel`，根据车辆几何参数计算 4 个舵轮的转角与转速，发布到 Gazebo 关节控制话题。
- `groundtruth_odom_publisher`：将 Gazebo 真值位姿转换为 `/groundtruth_odom` 及 TF（odom→base_link）。
- `world_handler`：加载不同世界模型（empty / maze / garden 等）及车辆 URDF。
- `joy_controller`：提供手柄遥控模式，在 Case 1 中可直接人工驾驶车辆。
- `map_visualizer`：对环境与路径进行 3D 可视化，便于调试。

---

## 3. 话题接口与数据流

### 3.1 主要话题（控制器视角）

**订阅：**

- 位姿/里程计：`/groundtruth_odom`（nav_msgs/Odometry）
- 全局路径：`/move_base/NavfnROS/plan`（nav_msgs/Path）
- 局部代价地图：`/move_base/local_costmap/costmap`（nav_msgs/OccupancyGrid）
- 参考代价图：
  - `/distance_error_map`（grid_map_msgs/GridMap）
  - `/ref_yaw_map`（grid_map_msgs/GridMap）

**发布：**

- 控制输出：`/cmd_vel`（geometry_msgs/Twist）
- 可视化：采样轨迹、最优轨迹、调试文本等 Marker/OverlayText
- 评估信息：自定义消息 `mppi_eval_msgs/MPPIEval`（记录代价分解、计算时间等）。

### 3.2 与 move_base / Gazebo 的关系

- 推荐入口已整理到 `src/bringup/noetic_container_bringup/launch/`，其中 `navigation.launch` 结构（核心部分）：
  1. 启动 `gazebo_world.launch`（Gazebo + world_handler + groundtruth_odom_publisher + joy_controller 等）；
  2. 启动 `map_server`、`amcl`、`map_visualizer`、`rviz`；
  3. 启动 `reference_costmap_generator`；
  4. 根据 `local_planner` 选择启动 `mppi_3d_a` / `mppi_3d_b`，并启动 `move_base`（仅保留其全局规划与 costmap 能力，不使用其本地控制输出）。

- 工作空间根目录 `launch/` 现在只保留兼容旧命令的薄包装入口，便于逐步迁移。

在做移植或改造时，主要是**保持这些关键话题与 TF 结构语义不变或在 launch 中改名映射**。

---

## 4. MPPI 算法要点与各变体差异

### 4.1 算法核心流程

1. 从当前状态出发，将控制序列离散为 T 个时间步（预测时域，例如 30 步，每步 0.03s 左右）。
2. 生成 K 条控制序列样本（典型 3000 条），其中：
   - 一部分围绕上一轮最优控制序列附近加噪声（exploitation）；
   - 一部分为纯噪声样本（exploration）。
3. 对每条控制序列，利用车辆运动学模型前向仿真，沿途累计阶段代价并加上终端代价：
   - 速度跟踪误差、航向对齐误差；
   - 碰撞代价（来自 costmap）；
   - 偏离参考路径的距离代价（来自 `/distance_error_map`）；
   - 控制输入变化/车辆命令变化代价；
   - 到目标点的终端距离罚。
4. 使用 Path-Integral 形式对所有样本加权：
   - 根据代价经温度参数 `lambda` 归一化，得到每条样本的权重；
   - 以样本噪声的加权和更新最优控制序列。
5. 可选：对得到的控制序列做 Savitzky-Golay 等滤波平滑。

### 4.2 各变体的设计取向

- **MPPI-3D(a)**：
  - 控制空间：$(v_x, v_y, \omega)$；
  - 线速度噪声较大 → 更倾向快速前进，绕障较“冲”；
  - 适合需要高效率但可接受一定风险的场景。

- **MPPI-3D(b)**：
  - 控制空间仍为 $(v_x, v_y, \omega)$；
  - 线速度噪声减小，角速度噪声相对增加 → 更积极地调整航向、收敛到参考路径；
  - 更保守安全，速度略慢。

---

## 5. 重要配置文件与调参入口

### 5.1 全局/局部 costmap（config/）

- `costmap_common_global.yaml` / `costmap_common_local.yaml`
  - `obstacle_range` / `raytrace_range`：障碍/射线追踪最大距离；
  - `robot_radius`：机器人近似半径（需根据实际底盘调整）；
  - `inflation_radius`：障碍膨胀半径，直接影响安全裕度；
  - `observation_sources` / `scan`：雷达话题名称、坐标系、是否用作 marking/clearing。

- `costmap_global.yaml` / `costmap_local.yaml`
  - 坐标系 `global_frame` / `robot_base_frame`；
  - `static_map` 与 `rolling_window`；
  - `width` / `height` / `resolution` 等窗口与分辨率参数。

- `move_base.yaml`
  - `controller_frequency` / `planner_frequency`；
  - 抖动/振荡检测参数（`oscillation_timeout` / `oscillation_distance` 等）。

### 5.2 定位与 SLAM

- `amcl.yaml`
  - 里程计与激光误差模型（`odom_alpha*`、`laser_*` 系列参数）；
  - `odom_model_type: omni` 适配全向/舵轮底盘；
  - 粒子数、更新阈值等影响收敛速度与稳定性。

- `gmapping.yaml`
  - 地图更新间隔、最大测距、步长、迭代次数、粒子数等。

### 5.3 MPPI 控制器内部参数

- 位置：对应控制包的源码中（如 `src/control/mppi_3d`下的 `param` / `*_setting` 文件）。
- 关键字段包括：
  - 采样与时域：`num_samples`、`prediction_horizon`、`step_len_sec`；
  - 信息论相关：`param_lambda`（温度参数）、`param_alpha`（信息混合）、`param_exploration`（探索比例）；
  - 噪声协方差：`sigma_*`（不同模式对应不同采样分布）；
  - 代价权重：`weight_velocity_error`、`weight_angular_error`、`weight_collision_penalty`、`weight_distance_error_penalty`、`weight_terminal_state_penalty`、`weight_vehicle_cmd_change` 等。

调参建议：

1. 先固定车辆模型与传感器正确性，再改 costmap 参数使路径/局部地图正常；
2. 然后通过调整 MPPI 权重平衡“走得快/离障远/贴近参考路径/控制平滑”等目标；
3. 最后视实时性能再调整 `num_samples` 与 `prediction_horizon`。

---

### 6.1 话题接口适配

- 输入适配：
  - 里程计：若真实车只发布 `/odom`，可在你自己的 launch 中将 `odom_topic` 参数设为 `/odom` 并在 `navigation.launch` 中 remap；
  - 激光：修改 `scan_topic` 与 costmap 中的 `topic`、`sensor_frame`，保证雷达数据与 TF 一致；
  - 全局路径：如果不用 `move_base/NavfnROS`，只要你的全局规划器发布 `nav_msgs/Path`，即可在 `reference_costmap_generator` 的 launch 中改为对应话题。

- 输出适配：
  - 适配 `/cmd_vel` 话题。

### 6.2 修改代价函数

- 在控制器内部 `*_setting` 或 cost 相关文件中，可以：
  - 调整现有代价项权重（速度、碰撞、距离、终端等）；
  - 新增与任务相关的代价项（例如对越障角度、车身姿态、能源消耗等约束）。
- 建议：
  - 初期先只新增小权重新项，观察行为变化；
  - 避免直接删除核心安全项（碰撞惩罚、距离惩罚等）。

### 6.3 修改控制约束与采样空间

- 修改最大速度/角速度：
  - 在通用控制类型（如 `VxVyOmega`）和车辆参数处更新对应上限；
  - 同步检查 `sigma`、权重和 costmap/inflation 半径是否仍匹配新的最大速度。

### 6.4 性能与实时性

- 影响因素：
  - `num_samples`、`prediction_horizon` 越大，计算量越高；
  - 是否开启轨迹可视化、日志输出；
  - 硬件 CPU 性能与 OpenMP 支持。

- 调优方向：
  - 若存在实时性瓶颈，可优先减小样本数与预测步数；
  - 关闭大部分可视化，仅保留必要调试信息；
  - 适当提高 `controller_frequency` 前先确保单次计算在周期内完成。

---

## 8. Saturn Lite 六足机器人集成方案

本节描述如何将 Saturn Lite 六足机器人模型集成到当前 MPPI 导航框架中，替代原有的 4WIDS 舵轮车。

### 8.1 架构说明：本项目使用 ROS1，而非 Nav2

首先需要明确：**本项目完全基于 ROS1 (Noetic) + catkin，使用的是 ROS1 的 `move_base` 导航框架，而不是 ROS2 的 Nav2**。

| 组件 | 本项目实际使用 | Nav2 (ROS2) 对应 |
|------|---------------|------------------|
| 构建系统 | catkin build | colcon build |
| 全局规划 | move_base + NavfnROS | Nav2 Planner Server |
| 局部规划 | **MPPI 控制器（旁路 move_base 局部规划）** | Nav2 Controller Server |
| 代价地图 | move_base costmap_2d | Nav2 Costmap2D |
| 定位 | amcl (ROS1) | Nav2 AMCL |
| SLAM | gmapping | slam_toolbox |

MPPI 控制器的核心设计思想：**完全旁路 `move_base` 的本地输出**（将其 `cmd_vel` remap 到 `/no_use/cmd_vel`），仅利用 `move_base` 的全局规划和 costmap 维护能力，由 MPPI 自主生成 `/cmd_vel` 控制指令。

### 8.2 Saturn Lite 六足机器人概述

| 参数 | 数值 |
|------|------|
| 自由度 | 18 DOF（6 条腿 × 3 关节：hip/thigh/shank） |
| 整机质量 | ~29.6 kg（机身 16.4 kg + 6 条腿） |
| 机身长度 | ~0.86 m（前后腿间距 0.664 m） |
| 站立高度 | 0.42 m |
| 最大速度 | ~0.5 m/s（步态限制） |
| 关节类型 | Hip: 髋横滚 (±0.67 rad), Thigh: 大腿俯仰 (-0.785~3.14), Shank: 小腿俯仰 (-2.68~0) |
| 运控方式 | RL 策略网络（ONNX）→ 18 关节目标位置 |

六条腿排布：
```
       FL ─────── FR          (前排: x=+0.333)
       │           │
  ML ──┼───────────┼── MR     (中排: x=+0.001)
       │           │
       RL ─────── RR          (后排: x=-0.331)
```

### 8.3 集成架构

集成后的数据流：

```
[Gazebo World] + [Saturn Lite URDF (含 Gazebo 插件)]
       │
       ├── p3d plugin ────→ /groundtruth_odom
       ├── lidar plugin ──→ /laser_link/scan
       ├── imu plugin ────→ /saturn/imu
       └── ros_control ───→ /saturn/joint_states
                           ← /saturn/*_controller/command (18 个)
       
[groundtruth_odom_publisher] → TF (odom→base_link)
[map_server] → /map
[amcl] → TF (map→odom)
[move_base] → /move_base/NavfnROS/plan, /move_base/local_costmap/costmap

[reference_costmap_generator] → /distance_error_map, /ref_yaw_map

        ↓ 全部输入 MPPI 控制器 ↓

[mppi_3d_node (hexapod config)] → /cmd_vel (vx, vy, omega)

        ↓

[hexapod_vel_driver.py] → 18 个关节位置命令
  (三足步态控制器)         → /saturn/*_controller/command
```

**关键设计原则**：MPPI 控制器将机器人视为可接收 `(vx, vy, omega)` 指令的"黑箱"。底层的轮式/足式差异由速度驱动节点（`vel_driver` / `hexapod_vel_driver`）处理。因此 MPPI 算法核心代码无需修改，只需调整参数。

### 8.4 新增文件结构

```
src/simulation/gazebo/saturn_gazebo/
├── CMakeLists.txt
├── package.xml
├── meshes/                          → 符号链接到 saturn_lab4.5/.../meshes/
├── scripts/
│   ├── process_urdf.py              # URDF 处理器：添加 Gazebo 插件
│   └── hexapod_vel_driver.py        # 三足步态速度驱动
├── config/
│   ├── saturn_controller.yaml       # 18 个关节的 ros_control 配置
│   └── hexapod_vel_driver.yaml      # 步态参数
└── launch/
    └── saturn_gazebo_world.launch   # Gazebo + 六足机器人完整启动

src/control/mppi_3d/
├── config/mppi_3d_hexapod.yaml      # MPPI 六足专用参数
└── launch/mppi_3d_hexapod.launch    # MPPI 六足启动

config/
├── costmap_common_local_hexapod.yaml    # 局部 costmap（robot_radius=0.45）
└── costmap_common_global_hexapod.yaml   # 全局 costmap

src/bringup/noetic_container_bringup/
├── launch/
│   ├── gazebo_world.launch          # 4WIDS Gazebo 启动
│   ├── navigation.launch            # 4WIDS 顶层导航启动
│   ├── gmapping.launch              # 4WIDS 建图启动
│   └── rosbag_play.launch           # rosbag 回放可视化
├── CMakeLists.txt
└── package.xml

launch/
└── navigation_hexapod.launch        # 六足导航顶层启动文件（当前仍保留在根目录）
```

### 8.5 使用方法

#### 1) 构建

```bash
cd ~/mppi_swerve_drive_ros
catkin build saturn_gazebo
# 或全量构建
catkin build
```

#### 2) 仅启动 Gazebo + 六足机器人

```bash
roslaunch saturn_gazebo saturn_gazebo_world.launch gazebo_world_name:=maze
```

#### 3) 完整导航（MPPI + 六足）

```bash
roslaunch navigation_hexapod.launch gazebo_world_name:=maze
```

在 RViz 中使用 `2D Nav Goal` 设定目标点，MPPI 控制器将自动规划并驱动六足机器人。

### 8.6 URDF 处理机制

原始 Saturn Lite URDF（由 SolidWorks 导出，1568 行）缺少 Gazebo 所需的组件。`process_urdf.py` 脚本自动添加：

| 添加内容 | 说明 |
|---------|------|
| `base_link` | 导航栈要求的根坐标系 |
| `laser_link` + Gazebo ray sensor | 2D 激光雷达（360°, 30m 量程） |
| 18 个 `<transmission>` | 每个 revolute 关节的 PositionJointInterface |
| `gazebo_ros_control` 插件 | 关节控制器管理（命名空间 `/saturn`） |
| `p3d` 插件 | 真值里程计 |
| IMU 传感器 | 200Hz 加速度/角速度 |
| 足端摩擦属性 | μ=1.0, kp=1e6, kd=100 |
| 各 link 材质 | Gazebo 可视化颜色 |

### 8.7 三足步态控制器

`hexapod_vel_driver.py` 实现了基于相位的三足步态（Tripod Gait）：

**分组**：
- Group A: FL, MR, RL — 同相位
- Group B: FR, ML, RR — 相位差 π

**关节映射**：
- `hip` (髋横滚): 控制侧向运动 → `vy` 映射
- `thigh` (大腿俯仰): 控制前后步幅 → `vx` + `omega` 映射
- `shank` (小腿俯仰): 控制脚面抬起高度 → 摆动相自动抬腿

**速度→关节映射**：

$$\theta_{thigh} = \theta_0 - A_{stride} \cdot \frac{v_x}{v_{max}} \cdot \cos(\phi) - K_{turn} \cdot \omega \cdot \cos(\phi)$$

$$\theta_{shank} = \begin{cases} \theta_0 + A_{lift} \cdot \sin(\phi) & \text{摆动相 (sin(φ)>0)} \\ \theta_0 & \text{支撑相} \end{cases}$$

$$\theta_{hip} = \theta_0 + K_{lat} \cdot v_y \cdot s_{side}$$

其中 $\phi$ 为步态相位，$s_{side}$ 为左右侧符号。

> **进阶：RL 控制器集成**
>
> Saturn Lite 自带预训练 RL 策略网络（`saturn_control.cpython-310-x86_64-linux-gnu.so`），可替代简易三足步态。该控制器接收 `[vx, vy, yaw_rate]` + 机器人状态 → 输出 18 关节目标位置，品质远高于固定步态。集成方法：
>
> 1. 确保 Python 3.10 + onnxruntime 环境与 ROS 环节兼容
> 2. 在 `hexapod_vel_driver.py` 中导入 `SaturnRobot` 类
> 3. 在 control_loop 中调用 `SaturnRobot.control()` 替代三足步态逻辑
> 4. 将 `/saturn/joint_states` 和 `/saturn/imu` 反馈传入控制器

---

## 9. 轮式 → 足式迁移注意事项

### 9.1 运动模型差异

| 维度 | 4WIDS 舵轮车 | 六足机器人 |
|------|-------------|-----------|
| 控制输入 | $(v_x, v_y, \omega)$ → 8D 车轮 | $(v_x, v_y, \omega)$ → 18D 关节 |
| 运动学约束 | 轮胎非完整约束、最大转向角 | 足端工作空间、运动学可达、稳定裕度 |
| 速度范围 | vx ∈ [-2, 2], vy ∈ [-2, 2] | vx ∈ [-0.5, 0.5], vy ∈ [-0.3, 0.3] |
| 运动平滑性 | 连续、无脉冲 | 离散步态、周期性脉动 |
| 地形能力 | 仅平地 | 可越障、爬坡、非结构化地形 |
| 里程计精度 | 轮速编码器（漂移小） | 足端打滑、IMU 积分漂移 |

### 9.2 MPPI 参数调整要点

迁移到六足机器人时，需要调整的 MPPI 参数：

**A. 速度约束（`common_type.hpp`）**
```cpp
// 原始（舵轮车）
static constexpr double VX_MAX = 2.0;
static constexpr double VY_MAX = 2.0;
static constexpr double OMEGA_MAX = 1.57;

// 六足调整
static constexpr double VX_MAX = 0.5;
static constexpr double VY_MAX = 0.3;
static constexpr double OMEGA_MAX = 1.0;
```

**B. 采样噪声（`sigma`）**
- 降低 σ 以匹配更窄的速度范围
- 推荐：`[0.35, 0.25, 0.65]`（原值 `[1.0, 1.0, 0.78]`）

**C. 参考速度**
- 从 2.0 m/s 降至 0.3~0.4 m/s

**D. 代价权重**
- `weight_collision_penalty`: 增大（50→80），足式机器人碰撞后恢复困难
- `weight_cmd_change`: 增大（0→2.0），减少控制跳变导致的步态不稳
- `weight_vehicle_cmd_change`: 全部置零（无车轮命令）

**E. 预测时域**
- `step_len_sec`: 增大到 0.05s（匹配步态控制周期）
- 总预测时长保持 ~1.5s

### 9.3 需要修改的代码模块

| 模块 | 修改内容 | 难度 |
|------|---------|------|
| `vel_driver` | 替换为 `hexapod_vel_driver`（已完成） | ★☆☆ |
| URDF 模型 | 新模型 + Gazebo 插件（已完成） | ★☆☆ |
| `mppi_3d_setting.hpp` | 可选：去除 8D 车轮代价项 | ★☆☆ |
| `common_type.hpp` | 修改速度限幅 | ★☆☆ |
| MPPI YAML 参数 | 新配置文件（已完成） | ★☆☆ |
| costmap 参数 | `robot_radius` 调整（已完成） | ★☆☆ |
| 里程计/定位 | 可能需要 IMU 融合（视觉惯性里程计） | ★★☆ |
| 代价函数 | 可选：添加地形感知代价项 | ★★★ |
| 步态控制器 | 简易→RL 策略网络 | ★★★ |

### 9.4 Costmap 与碰撞检测

六足机器人的碰撞模型比轮式车更复杂：
- **足展包络**: 行走时腿部横向展开约 0.35m
- **摆动包络**: 摆动腿可能前伸/后摆约 0.15m
- **建议**: `robot_radius` 设为 0.45m（保守值），`inflation_radius` 设为 0.50m

### 9.5 TF 树兼容性

迁移后需保持 TF 树结构不变：
```
map → odom → base_link → base_body → (各 link)
                       → laser_link
```
`base_link` 由固定关节连接到 `base_body`（机身），确保导航栈（amcl / move_base / costmap）能正确查找机器人位姿。

---

## 10. MPPI 算法深入优化指南

以下内容旨在为深入理解和优化 MPPI 控制器提供系统性的思考框架。

### 10.1 当前实现的核心数学形式

**信息论 MPPI (Path Integral)**

给定当前状态 $x_0$、控制序列 $\mathbf{u} = \{u_0, \ldots, u_{T-1}\}$，MPPI 通过以下步骤迭代优化：

1. **采样**：生成 $K$ 条扰动 $\{\epsilon_k\}_{k=1}^K$，$\epsilon_k \sim \mathcal{N}(0, \Sigma)$
2. **前向仿真**：对每条采样
   $$\tilde{u}_k = \begin{cases} u^*_{prev} + \epsilon_k & \text{exploitation（前 90\%）} \\ \epsilon_k & \text{exploration（后 10\%）}\end{cases}$$
3. **代价累积**：
   $$S_k = \sum_{t=0}^{T-1} \left[ q(x_t^k, \tilde{u}_t^k) + \lambda \tilde{u}_t^{k\top} \Sigma^{-1} \epsilon_t^k \right] + \phi(x_T^k)$$
4. **权重计算**：
   $$w_k = \frac{1}{\eta} \exp\left(-\frac{1}{\lambda}(S_k - S_{min})\right), \quad \eta = \sum_k \exp\left(-\frac{1}{\lambda}(S_k - S_{min})\right)$$
5. **控制更新**：
   $$u^* \leftarrow u^*_{prev} + \sum_{k=1}^K w_k \cdot \epsilon_k$$

### 10.2 温度参数 λ 的深层理解

$\lambda$ 是整个 MPPI 最关键的超参数之一：

- **λ → 0**：权重极端分化，趋近贪心选择（仅最优样本），方差大
- **λ → ∞**：权重趋于均匀，更新保守，可能不收敛
- **实践观察**：
  - λ=200（当前值）适合高速场景（样本代价范围大）
  - 低速六足场景建议 λ=100~150（代价范围更小，需更细腻的区分）

**优化方向：自适应 λ**
```
λ_adaptive = λ_0 × (S_max - S_min) / S_reference
```
根据当前样本代价范围动态调整温度，避免权重过于集中或过于分散。

### 10.3 采样效率优化

#### A. 协方差矩阵自适应

当前 sigma 为固定对角阵 `[σ_vx, σ_vy, σ_ω]`。优化方向：

- **时变协方差**：预测初期用大 σ（探索），后期用小 σ（收敛）
- **状态相关协方差**：接近障碍物时增大角速度噪声，开阔区域增大线速度噪声
- **协方差矩阵学习**：基于历史轨迹代价梯度自适应更新 Σ（CMA-MPPI 方法）

$$\Sigma_{t+1} \propto \sum_k w_k \cdot \epsilon_k \epsilon_k^\top$$

#### B. importance sampling 优化

当前的 exploitation/exploration 分割（90%/10%）是固定的。可改进为：

- 基于上一步有效样本数（ESS = $1/\sum w_k^2$）动态调整探索比例
- ESS 低 → 增大探索比例
- ESS 高 → 减少探索，集中利用

#### C. 分层采样 (Stratified Sampling)

将采样空间分为若干子区域，在每个子区域内等比例采样，减小方差。

### 10.4 代价函数设计哲学

#### A. 当前代价项分析

```
总代价 = 速度跟踪 + 航向对齐 + 碰撞惩罚 + 路径偏离 + 控制平滑 + 终端代价
```

**问题与改进方向**：

1. **碰撞代价为软约束**：collision_costmap 值 × 权重，可能在极端情况下被其他项"冲掉"
   - 改进：使用 barrier function 或 log-barrier 使碰撞代价在近距离时趋于无穷
   
   $$q_{collision}(d) = \begin{cases} w_{coll} \cdot C(x,y) & d > d_{safe} \\ +\infty & d \leq d_{safe} \end{cases}$$

2. **速度跟踪方向依赖**：当前的 ref_aligned_vel 投影较好，但在路径拐角处可能出现目标方向突变
   - 改进：使用前瞻（look-ahead）参考点而非当前最近点

3. **缺少横向加速度约束**：对足式机器人尤其重要（侧倾稳定性）
   - 新增项：$w_{lat} \cdot (v_y + \omega \cdot v_x / g)^2$ 限制等效侧倾角

#### B. 足式机器人专用代价项（进阶）

```
# 地形感知代价（需要高程图输入）
terrain_cost = w_terrain × roughness(x, y)

# 步态稳定性代价（需要支撑多边形信息）
stability_cost = w_stability × max(0, margin_threshold - stability_margin)

# 能耗代价
energy_cost = w_energy × Σ|τ_i × ω_i|  # 关节力矩 × 角速度

# 高度一致性代价（保持机身水平）
posture_cost = w_posture × (roll² + pitch²)
```

### 10.5 运动模型改进

#### A. 当前模型（单质点运动学）

$$x_{t+1} = x_t + v_x \cos\psi \cdot dt - v_y \sin\psi \cdot dt$$
$$y_{t+1} = y_t + v_x \sin\psi \cdot dt + v_y \cos\psi \cdot dt$$
$$\psi_{t+1} = \psi_t + \omega \cdot dt$$

这是适用于全向移动机器人的简化模型。对于足式机器人，可考虑以下改进：

#### B. 加速度/动力学约束

在前向仿真中加入加速度限制：
$$v_{x,t+1} = v_{x,t} + \text{clamp}(\dot{v}_x, -a_{max}, a_{max}) \cdot dt$$

这能更真实地反映六足机器人的加减速特性。

#### C. 步态引起的速度波动

足式机器人的实际速度会因步态周期产生周期性波动，可建模为：
$$v_{actual} = v_{cmd} \cdot (1 + A_{gait} \sin(2\pi f_{gait} t))$$

在前向仿真中加入此修正项可提高预测精度。

#### D. 模型学习（数据驱动）

用 GP (高斯过程) 或神经网络学习实际 cmd_vel → 状态变化的映射，替代解析运动学模型，可显著提高预测准确度。

### 10.6 计算性能优化

| 优化方向 | 方法 | 预期加速比 |
|---------|------|-----------|
| GPU 并行 | CUDA/OpenCL 并行采样与前向仿真 | 5~20× |
| SIMD 向量化 | AVX2/AVX-512 向量化运动学计算 | 2~3× |
| 减少 GridMap 查询 | 预计算变换矩阵，批量查询 | 1.5~2× |
| 自适应采样数 | 简单场景减少 K，复杂场景增加 K | ~1.5× 平均 |
| 异步流水线 | 采样/仿真/权重计算流水线化 | ~1.3× |
| 降低预测频率 | 当误差小时降低控制器更新频率 | ~2× 平均 |

当前实现使用 **OpenMP** 并行 3000 条采样，已有一定优化基础。进一步提升需考虑 GPU 路线。

### 10.7 鲁棒性增强

#### A. Tube-MPPI

在标称 MPPI 轨迹周围定义安全管道（tube），当实际状态偏离管道时触发安全控制器：

$$u = u_{MPPI} + K_{feedback}(x_{actual} - x_{nominal})$$

#### B. 多模态 MPPI

维护多个控制序列分布（高斯混合模型），避免在多条可行路径间振荡：

$$p(u) = \sum_{m=1}^{M} \pi_m \mathcal{N}(u | \mu_m, \Sigma_m)$$

#### C. 约束 MPPI

将硬约束（如碰撞避免、速度限制）显式纳入优化：
- 投影法：将违反约束的样本投影到可行集
- 增广拉格朗日法：在代价函数中加入约束乘子项

### 10.8 MPPI 从舵轮车迁移到六足的优化总结

```
Step 1: 参数层面（已完成）
  ├── 降低速度限幅和参考速度
  ├── 调整采样噪声 sigma
  ├── 调整代价权重（去掉 8D 车轮代价）
  └── 调整 costmap robot_radius

Step 2: 系统集成层面（已完成）
  ├── 新建 hexapod_vel_driver 替代 vel_driver
  ├── 处理 URDF（传感器、执行器、Gazebo 插件）
  └── launch 文件适配

Step 3: 控制品质优化（推荐）
  ├── 增大 weight_cmd_change 减少控制跳变
  ├── 增加加速度约束模型
  ├── 调整 step_len_sec 匹配步态周期
  └── 优化 Savitzky-Golay 窗口

Step 4: 深度优化（探索方向）
  ├── 自适应 λ 温度参数
  ├── 协方差矩阵自适应
  ├── 地形感知代价项
  ├── 数据驱动运动模型
  └── GPU 加速
```

### 10.9 推荐学习路径

1. **理解基础**：阅读 `mppi_3d_core.cpp` 的 `solveMPPI()` 函数，理解完整 MPPI 流程
2. **理解代价函数**：阅读 `mppi_3d_setting.hpp` 中的 `stage_cost()` 和 `terminal_cost()`
3. **理解运动学**：阅读 `calcNextState()` 和 `convertControlSpace3DToControlSpace8D()`
4. **对比变体**：比较 `mppi_3d_a.yaml` 和 `mppi_3d_b.yaml` 的参数差异，在仿真中观察行为差别
5. **动手调参**：修改 `mppi_3d_hexapod.yaml` 中的权重和 sigma，观察 MPPI 轨迹分布变化
6. **进阶改造**：尝试在 `stage_cost()` 中添加新的代价项（如侧向加速度惩罚）
7. **论文参考**：阅读 IROS 2024 原论文 "*Switching Sampling Space of MPPI Controller to Balance Efficiency and Safety in 4WIDS Vehicle Navigation*"

### 10.10 关键参考资料

- **MPPI 原始论文**: Williams et al., "Information Theoretic MPC for Model-Based Reinforcement Learning" (ICRA 2017)
- **本项目论文**: Aoki et al., "Switching Sampling Space of MPPI Controller..." (IROS 2024)
- **MPPI-Generic**: Vlahov et al., "MPPI-Generic: A CUDA Library for Stochastic Optimization" (RSS 2024)
- **Tube-MPPI**: Gandhi et al., "Robust Model Predictive Path Integral Control: Analysis and Performance Guarantees"
- **CMA-MPPI**: Yin et al., "CMA-ES based MPPI" (协方差自适应)
- **足式机器人 MPC**: Di Carlo et al., "Dynamic Locomotion in the MIT Cheetah 3 Through Convex MPC" (IROS 2018)

如后续在“对接自有机器人”“替换定位模块”“移植到 ROS2/Nav2”方面有更具体的需求，可以针对某一子任务再细化设计与实现步骤。

 我已经按源码主线把 mppi_3d_core 梳理完了。先给你结论：这个控制器的核心不是“六足动力学 
  MPPI”，而是一个基于 vx, vy, omega 机体速度指令的二维平面 MPPI
  局部规划/控制器；六足版本目前主要是参数适配 + 下层步态驱动承接 cmd_vel。

  1. 整体调用链

  入口在 src/control/mppi_3d/src/mppi_3d.cpp。ROS 层负责订阅 odom、ref_path、collision_costmap、
  distance_error_map、ref_yaw_map，然后定时调用 mppi_core_->solveMPPI(...)，最后发布 
  cmd_vel、最优轨迹、采样轨迹和评估消息。

  所以职责很清楚：

   - mppi_3d.cpp：ROS 封装、收发消息
   - mppi_3d_core.cpp：MPPI 优化主循环
   - mppi_3d_setting.hpp：预测模型和代价函数定义

  2. solveMPPI() 到底在做什么

  MPPICore::solveMPPI() 就是完整的 MPPI 一轮优化。

  第一步先判断是否到达目标。若位置和朝向都在容差内，直接输出零速度。

  第二步初始化本轮代价，并按需要生成噪声矩阵 noises_[k][t][u]。这里 K 是样本数，T 是预测时域，u 
  是控制维度 vx, vy, omega。

  第三步进入最核心的并行 rollout。对每个样本 k：

   - 从当前观测状态出发
   - 对每个预测步 t
   - 构造采样控制 u_samples_[k][t]

  这里有两种采样：

   - 前 (1 - exploration)
    * K 个样本：围绕上一轮最优控制序列 u_opt_seq_latest_ 加噪声，偏 exploitation
   - 后面一部分样本：直接用纯噪声，偏 exploration

  第四步用 calcNextState() 做状态传播。它的模型非常简单，本质是：

  x_{t+1} = x_t + (body velocity rotated to world) * dt

  也就是说，它假设机器人能直接执行 vx, vy, omega，这是全向底盘/速度层模型，不是足端接触模型。

  第五步累计代价：

   - stage_cost(...)
   - 一个 MPPI 里的控制相关项
   - 最后加 terminal_cost(...)

  第六步把所有样本代价做 softmax 型加权：

  w_k = exp(-(J_k - J_min)/lambda) / eta

  这里减去 J_min 是标准数值稳定技巧。

  第七步用加权噪声修正控制序列：

  u_opt_seq[t] = u_opt_seq_latest_[t] + sum_k w_k * noise[k][t]

  这就是 MPPI 的关键更新，不是梯度下降，而是“好样本更有话语权”的采样加权平均。

  第八步可选地对首个控制做 Savitzky-Golay 滤波，再做 clamp，最后只输出 u_opt_seq[0] 
  作为当前时刻控制。

  3. 这份实现的代价函数在鼓励什么

  mppi_3d_setting.hpp 里最重要。

  stage_cost() 主要有 6 类项：

   - 速度跟踪：让当前速度沿参考路径方向的投影接近目标速度
   - 朝向对齐：让 yaw 接近参考路径朝向
   - 碰撞代价：直接读 collision_costmap
   - 路径距离误差：读 distance_error_map
   - 控制变化惩罚：限制 vx, vy, omega 跳变
   - 8D 车辆执行器变化惩罚：把 3D 速度映射成 4 个舵角 + 4 个轮速，再惩罚执行器变化

  terminal_cost() 则更简单，就是还没接近目标时，惩罚末端状态离目标太远。

  这说明它不是“纯避障器”，而是同时在做：

   - 沿参考路径走
   - 保持合理朝向
   - 不撞
   - 控制别太抖

  4. 为什么说它适合做六足外层导航，但不是真正六足 MPPI

  适合的地方：

   - MPPI 主框架本身可复用
   - 输入输出是导航常见的 odom + path + costmap -> cmd_vel
   - 如果六足底层已有步态控制器能跟踪 cmd_vel，它可以做外层局部规划器

  不够的地方：

   - 状态没有质心高度、姿态滚转俯仰、支撑相位、触地点
   - 控制没有步频、步长、摆腿相位、足端力
   - 预测模型没有接触切换与动力学约束
   - 代价里没有稳定裕度、可达落脚点、地形可通行性

  所以它更像：把六足近似成可控的全向速度体，然后让下层步态去实现速度命令。

  5. a / b / hexapod 三套配置怎么理解

  mppi_3d_a.yaml 比较激进，sigma 里的平移噪声更大，所以采样云更发散，更容易找快但险的轨迹。

  mppi_3d_b.yaml 把平移噪声降下来了，角速度噪声相对更高，通常会更保守，转向修正更多，速度感更稳。

  mppi_3d_hexapod.yaml 明显是在做六足外层适配：

   - 更低参考速度
   - 更长 step_len_sec
   - 更高控制变化惩罚
   - 更高碰撞惩罚
   - 把 8D 车轮执行器代价清零

  这很合理，因为六足不该再用“舵角/轮速平滑”那套成本。

  6. 两个很值得你注意的源码细节

  第一，参数名可能有 bug。

  代码读取的是 controller/ref_velocity，但 yaml 和 README 写的是 reference_velocity
  。这意味着配置文件里改了参考速度，代码可能根本没读到，仍在用默认值 
  2.0。对六足尤其危险，因为你明明想要 0.35 m/s。

  第二，噪声生成里 OpenMP 并行共享了一个 std::mt19937。这在线程安全上是不严谨的，可能导致随机性异
  常。算法思想不受影响，但实现质量上这是个点。

  7. 一个额外的实现观察

  这份代码没有显式做很多 MPPI 实现里的“控制序列左移一位再补末尾”的 warm 
  start，而是直接把本轮优化得到的整段序列存回 u_opt_seq_latest_。它也能工作，但和经典
  receding horizon 的写法不完全一样。

  8. 如果你现在要把它用于足式机器人，我的建议是

  短期可以直接用它做外层速度型局部导航器。

  中期至少该补：

   - 加速度/速度变化约束
   - 地形代价
   - 机体姿态稳定性代价
   - 与步态周期一致的 dt 和时域

  长期如果要做真正“足式 MPPI”，就要把 state、control、calcNextState() 全部换成足式系统模型。
