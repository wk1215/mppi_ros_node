# mppi_swerve_drive_ros
**用于舵轮底盘导航的 MPPI（Model Predictive Path-Integral）控制器**

<div align="center">

![image](https://github.com/user-attachments/assets/56d055e7-f3a4-4c89-940f-577b00e4f088)

[[项目主页]](https://mizuhoaoki.github.io/projects/iros2024)
[[论文 PDF]](https://mizuhoaoki.github.io/media/papers/IROS2024_paper_mizuhoaoki.pdf)
[[Arxiv]](https://arxiv.org/abs/2409.08648)
[[IEEE Xplore]](https://ieeexplore.ieee.org/document/10802359)
[[Poster]](https://mizuhoaoki.github.io/projects/iros2024_poster.pdf)

[![ROS Distro: Noetic](https://img.shields.io/badge/ROS-Noetic-red.svg)](https://wiki.ros.org/noetic)
[![Docker](https://img.shields.io/badge/-Docker-EEE.svg?logo=docker&style=flat)](https://www.docker.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![Conference: IROS](https://img.shields.io/badge/Publication-IROS2024-purple.svg)](https://iros2024-abudhabi.org/)

![eyecatch_anim_3drviz](./media/eyecatch_anim_3drviz.gif)

</div>

## 引用

如果你在学术工作中使用了本项目，请引用以下论文：

```bibtex
@inproceedings{mizuho2024iros,
  author={Aoki, Mizuho and Honda, Kohei and Okuda, Hiroyuki and Suzuki, Tatsuya},
  booktitle={2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  title={Switching Sampling Space of Model Predictive Path-Integral Controller to Balance Efficiency and Safety in 4WIDS Vehicle Navigation},
  year={2024},
  volume={},
  number={},
  pages={3196-3203},
  doi={10.1109/IROS58592.2024.10802359}}
```

> [!IMPORTANT]
> **更高级的控制器也已开源**
> 可参考 **[Nullspace MPC Repository](https://github.com/MizuhoAOKI/nullspace_mpc)**。
> 与 MPPI 相比，它通常能够提供**更快的导航速度**和**更高的安全性**，但计算开销也会更大。

## 环境搭建

### 方式 1：Docker 环境

<details>
<summary>点击展开</summary>

1. 前置依赖
   - 安装 [docker](https://docs.docker.com/engine/install/ubuntu/)
   - Ubuntu 用户可参考：
     ```bash
     curl -fsSL https://get.docker.com -o get-docker.sh
     sudo sh get-docker.sh
     ```

2. 克隆仓库
   ```bash
   cd <你的工作空间路径>
   git clone https://github.com/MizuhoAOKI/mppi_swerve_drive_ros
   ```

3. 构建并启动 Docker 容器
   ```bash
   cd <你的工作空间路径>/mppi_swerve_drive_ros
   docker compose build
   docker compose up -d
   ```

4. 进入容器
   ```bash
   cd <你的工作空间路径>/mppi_swerve_drive_ros
   docker compose exec noetic bash
   ```

5. 在容器内编译项目
   ```bash
   cd ~/mppi_swerve_drive_ros
   source /opt/ros/noetic/setup.bash
   catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O2"
   source devel/setup.bash
   ```

</details>

### 方式 2：原生环境

<details>
<summary>点击展开</summary>

1. 前置依赖
   - [ubuntu 20.04](https://releases.ubuntu.com/focal/)
   - [ros noetic](https://wiki.ros.org/noetic)

2. 克隆仓库
   ```bash
   cd <你的工作空间路径>
   git clone https://github.com/MizuhoAOKI/mppi_swerve_drive_ros
   ```

3. 安装基础依赖
   ```bash
   cd <你的工作空间路径>/mppi_swerve_drive_ros
   sudo make install_deps
   ```

4. 初始化 rosdep 并安装依赖
   ```bash
   cd <你的工作空间路径>/mppi_swerve_drive_ros
   sudo rosdep init
   rosdep update
   rosdep install -y --from-paths src --ignore-src --rosdistro noetic
   ```

5. 编译项目
   ```bash
   cd <你的工作空间路径>/mppi_swerve_drive_ros
   make build
   ```

</details>

## 构建

编译整个项目：

```bash
cd <你的工作空间路径>/mppi_swerve_drive_ros
make build
```

如有必要，可先清理缓存再重新编译：

```bash
cd <你的工作空间路径>/mppi_swerve_drive_ros
make clean
```

## 使用方法

### 场景 1：仅启动 Gazebo 仿真，并通过手柄手动控制 4WIDS 小车

```bash
cd <你的工作空间路径>/mppi_swerve_drive_ros
source /opt/ros/noetic/setup.bash && source ./devel/setup.bash
roslaunch mppi_bringup gazebo_world.launch gazebo_world_name:=maze
```

- `gazebo_world_name` 可选值：
  - `empty`
  - `empty_garden`
  - `cylinder_garden`
  - `maze`
- 默认手柄设备路径为 `/dev/input/js0`。如果需要修改，请编辑 `src/operation/joy_controller/config/joy.yaml`。

### 场景 2：使用 MPPI 控制器进行自主导航

> [!NOTE]
> 工作空间根目录 `launch/*.launch` 目前仍保留为兼容旧命令的薄包装入口。
> 推荐直接使用 `mppi_bringup` 功能包中的启动文件。

- 尝试 `MPPI-3D(a)`（更激进，速度更快，但风险更高）
  ```bash
  cd <你的工作空间路径>/mppi_swerve_drive_ros
  source /opt/ros/noetic/setup.bash && source ./devel/setup.bash
  roslaunch mppi_bringup navigation.launch local_planner:=mppi_3d_a
  ```

- 尝试 `MPPI-3D(b)`（更保守、更平稳，但速度更慢）
  ```bash
  cd <你的工作空间路径>/mppi_swerve_drive_ros
  source /opt/ros/noetic/setup.bash && source ./devel/setup.bash
  roslaunch mppi_bringup navigation.launch local_planner:=mppi_3d_b
  ```

https://github.com/user-attachments/assets/eaeb7713-c09f-4a68-b68c-100444438f9e

> [!NOTE]
> 由于 ROS / Gazebo 仿真是异步的，而采样型算法又依赖多线程计算，不同用户环境下控制器表现会存在一定差异。

## MPPI 算法核心

- 状态空间为 `x, y, yaw`
- 控制空间为 `vx, vy, omega`
- 每个控制周期内，MPPI 会在 `num_samples × prediction_horizon` 的控制序列上进行采样
- 每条采样控制序列都会通过运动学模型前向 rollout，并累计阶段代价与终端代价
- 之后根据代价计算样本权重，再重建下一轮最优控制序列
- 输出的首个控制量会发布为 `/cmd_vel`

当前阶段代价主要包含：

- 参考路径方向上的速度跟踪误差
- 航向对齐误差
- 碰撞代价
- 路径距离误差
- 控制变化惩罚
- 横向速度惩罚

终端代价主要用于避免预测终点偏离目标过远。

## 当前 MPPI 优化内容

当前 `noetic` 分支在原始 MPPI 基础上，已经加入了更适合上层导航控制与实际调试的若干优化：

- 修复了 `reference_velocity` 参数加载问题，确保 YAML 中的目标速度真正生效
- 新增了 `vx / vy / omega` 的可配置限幅，而不是只依赖源码里的硬编码限制
- 增加了基于 `weight_cmd_change` 的控制变化惩罚，改善命令抖动
- 增加了 `weight_lateral_velocity_penalty`，抑制不必要的横向速度
- 在 `mppi_3d_a` / `mppi_3d_b` 配置中关闭了与舵轮 8 维轮系命令直接相关的惩罚，更适合只输出 `/cmd_vel` 的上层控制模式
- 重写了最优控制序列的更新方式：现在基于**真实评估过的采样控制序列**重建 nominal，而不是简单叠加噪声
- 新增 `best_sample_blend`，允许适度向最优样本靠拢，减弱多模态平均带来的摆头与犹豫
- 修复了 warm-start 逻辑：应用首个控制后，会将控制序列按 receding horizon 向前平移
- 修复了 OpenMP 下共享随机数引擎导致的不稳定采样问题，改为每个 sample 独立随机数引擎
- 对控制历史与滤波缓存做了显式零初始化，减少启动初期的跳变
- 保留 Savitzky-Golay 滤波作为最终首拍控制平滑步骤
- 去除了“靠近目标点时按距离强制减速”的显式启发式，让减速行为更多由 MPPI 求解本身决定

## 关键调参参数

对新的轮式平台进行适配时，通常最值得优先调整的是：

- `reference_velocity`
- `sigma`
- `param_exploration`
- `param_lambda`
- `vx_min / vx_max / vy_min / vy_max / omega_min / omega_max`
- `weight_cmd_change`
- `weight_collision_penalty`
- `weight_distance_error_penalty`
- `weight_angular_error`
- `weight_terminal_state_penalty`
- `best_sample_blend`

一个简单的调参顺序建议是：

1. 先确认里程计、雷达、TF 和 costmap 正常
2. 再调整 `reference_velocity`、限幅、`sigma`
3. 然后调整碰撞、路径误差、航向误差和终端代价
4. 最后再微调 `best_sample_blend`、平滑窗口和探索强度
