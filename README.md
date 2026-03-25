# mppi_swerve_drive_ros
**MPPI (Model Predictive Path-Integral) Controller for a Swerve Drive Robot**

<div align="center">

![image](https://github.com/user-attachments/assets/56d055e7-f3a4-4c89-940f-577b00e4f088)

[[Website]](https://mizuhoaoki.github.io/projects/iros2024)
[[PDF]](https://mizuhoaoki.github.io/media/papers/IROS2024_paper_mizuhoaoki.pdf)
[[Arxiv]](https://arxiv.org/abs/2409.08648)
[[IEEE Xplore]](https://ieeexplore.ieee.org/document/10802359)
[[Poster]](https://mizuhoaoki.github.io/projects/iros2024_poster.pdf)

[![ROS Distro: Noetic](https://img.shields.io/badge/ROS-Noetic-red.svg)](https://wiki.ros.org/noetic)
[![Docker](https://img.shields.io/badge/-Docker-EEE.svg?logo=docker&style=flat)](https://www.docker.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![Conference: IROS](https://img.shields.io/badge/Publication-IROS2024-purple.svg)](https://iros2024-abudhabi.org/)

<!-- 1-min presentation movie -->
<!-- https://github.com/user-attachments/assets/0b18bb4d-c6ad-407b-919c-c8c3c219d75c -->

<!-- eyecatch movie -->
![eyecatch_anim_3drviz](./media/eyecatch_anim_3drviz.gif)

</div>

## Citation
If you use this work in an academic context, please cite the following publication:
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
> **More Advanced Controller is Also Open Source!**  
> Check **[Nullspace MPC Repository](https://github.com/MizuhoAOKI/nullspace_mpc)**  
> Compared to MPPI, it achieves **faster navigation** while ensuring **higher safety**,  
> at the cost of **increased computational demand**.

## Setup

### [Option 1] Docker environment

<details>
<summary>CLICK HERE TO EXPAND</summary>

1. Prerequisites
    - [docker](https://docs.docker.com/engine/install/ubuntu/)
        - For ubuntu users:
            ```
            curl -fsSL https://get.docker.com -o get-docker.sh
            sudo sh get-docker.sh
            ```

1. Clone the project repository.
    ```
    cd <path to your workspace>
    git clone https://github.com/MizuhoAOKI/mppi_swerve_drive_ros
    ```

1. Build and start the docker container.
    ```
    cd <path to your workspace>/mppi_swerve_drive_ros
    docker compose build
    docker compose up -d
    ```

1. Get into the docker container.
    ```
    cd <path to your workspace>/mppi_swerve_drive_ros
    docker compose exec noetic bash
    ```

1. [Inside the docker container] Build the project.
    ```
    cd ~/mppi_swerve_drive_ros
    source /opt/ros/noetic/setup.bash
    catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O2"
    source devel/setup.bash
    ```

</details>


### [Option 2] Native environment

<details>
<summary>CLICK HERE TO EXPAND</summary>

1. Prerequisites
    - [ubuntu 20.04](https://releases.ubuntu.com/focal/)
    - [ros noetic](https://wiki.ros.org/noetic)

1. Clone the project repository.
    ```
    cd <path to your workspace>
    git clone https://github.com/MizuhoAOKI/mppi_swerve_drive_ros
    ```

1. Install foundation packages.
    ```
    cd <path to your workspace>/mppi_swerve_drive_ros
    sudo make install_deps
    ```
1. Initialize rosdep, update it, and install dependencies.
    ```
    cd <path to your workspace>/mppi_swerve_drive_ros
    sudo rosdep init
    rosdep update
    rosdep update && rosdep install -y --from-paths src --ignore-src --rosdistro noetic
    ```
1. Build the project.
    ```
    cd <path to your workspace>/mppi_swerve_drive_ros
    make build
    ```

</details>  


## Build

Build the project.
```
cd <path to your workspace>/mppi_swerve_drive_ros
make build
```

(Optional) Clean the cache before building the project if necessary.
```
cd <path to your workspace>/mppi_swerve_drive_ros
make clean
```


## Usage

### [Case 1] Launch gazebo simulator only, operating a 4wids vehicle manually with a joypad.
```bash
cd <path to your workspace>/mppi_swerve_drive_ros
source /opt/ros/noetic/setup.bash && source ./devel/setup.bash
roslaunch mppi_bringup gazebo_world.launch gazebo_world_name:=maze
```

- `gazebo_world_name` options:
    - `empty`
    - `empty_garden`
    - `cylinder_garden`
    - `maze`
- Default joystick path is `/dev/input/js0`. If you want to change the path, please edit `mppi_swerve_drive_ros/src/operation/joy_controller/config/joy.yaml`.


### [Case 2] Navigate a 4wids vehicle autonomously with a MPPI controller.

> [!NOTE]
> Workspace-root `launch/*.launch` files are retained as compatibility wrappers.
> The recommended entrypoints now live in the `mppi_bringup` package.

- Try MPPI-3D(a) (driving faster but dangerous sometimes)
    ```bash
    cd <path to your workspace>/mppi_swerve_drive_ros
    source /opt/ros/noetic/setup.bash && source ./devel/setup.bash
    roslaunch mppi_bringup navigation.launch local_planner:=mppi_3d_a
    ```
- Try MPPI-3D(b) (relatively safe but driving slower)
    ```bash
    cd <path to your workspace>/mppi_swerve_drive_ros
    source /opt/ros/noetic/setup.bash && source ./devel/setup.bash
    roslaunch mppi_bringup navigation.launch local_planner:=mppi_3d_b
    ```

## MPPI Algorithm Core

- The controller state is `x, y, yaw`, and the control space is `vx, vy, omega`.
- At every control cycle, MPPI samples `num_samples x prediction_horizon` control sequences, rolls them out with the kinematic model, evaluates stage cost and terminal cost, and then reconstructs the next control sequence from the weighted sampled controls.
- The stage cost mainly contains velocity tracking along the reference path, heading alignment, collision cost, distance-to-path cost, and command change penalty.
- The terminal cost keeps the predicted terminal state from drifting too far away from the goal.

## Current MPPI Optimizations

The current Noetic branch includes the following practical improvements for more stable upper-layer navigation:

- Fixed the `reference_velocity` parameter loading so YAML speed settings take effect correctly.
- Added configurable command limits for `vx`, `vy`, and `omega`, instead of relying only on hard-coded limits.
- Added direct command smoothness penalties through `weight_cmd_change`, plus lateral velocity suppression through `weight_lateral_velocity_penalty`.
- Disabled the swerve-specific 8D wheel command penalty in the tuned `mppi_3d_a` / `mppi_3d_b` configs, which is more suitable when only `/cmd_vel` is used as the upper-layer output.
- Reworked the MPPI control update to reconstruct the nominal sequence from **evaluated sampled controls**, then optionally blend toward the best sample with `best_sample_blend`, which helps reduce mode averaging and oscillation.
- Fixed the warm-start behavior by shifting the control sequence forward in receding-horizon fashion after applying the first command.
- Fixed unstable random sampling under OpenMP by using per-sample random engines instead of a shared generator.
- Initialized control histories explicitly so the first few control cycles are less likely to jump due to uninitialized values.
- Kept the Savitzky-Golay filter as the final command smoothing step for the first control in the optimized sequence.
- Removed the explicit near-goal slowdown heuristic so deceleration is decided by MPPI optimization itself rather than by a hand-crafted distance rule.

### Main Parameters for Tuning

For adapting to a new wheeled platform, the most important parameters are usually:

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

https://github.com/user-attachments/assets/eaeb7713-c09f-4a68-b68c-100444438f9e

> [!NOTE]
> Due to asynchronous simulation on ROS and the sampling-based algorithm relying on multi-threading computation, the controllers' performance can vary depending on a user's environment.
