# ROS 2 PyBullet IIWA Workspace

This repository contains a ROS 2 Humble workspace for KUKA iiwa simulation in PyBullet, joint-space motion control, robot-state monitoring, RGB-D camera simulation, and a perception loop that turns segmented image regions into 3D targets.

Current archived patch release: `v0.1.1`

Current feature branch in progress: `feature/llm-task-execution`

The maintained path in this branch is the main chain:

```text
iiwa_pybullet_sim_node or iiwa_pybullet_rgbd_sim_node
  -> /iiwa7/joint_states, /iiwa7/init_done
iiwa_line_ik_desired or iiwa_keypoint_tracker_node
  -> /iiwa7/joint_desired
iiwa_impedance_controller
  -> /iiwa7/joint_torques
robotstate_bridge
  -> /robot_states
robot_monitor
```

For the RGB-D perception loop:

```text
iiwa_pybullet_rgbd_sim_node
  -> /sim/camera/*
sam3_mask_node
  -> /sam3/mask, /sam3/score
mask_depth_fusion_node
  -> /perception/keypoint_3d, /perception/keypoint_candidates, /perception/valid
iiwa_keypoint_tracker_node
  -> /iiwa7/joint_desired, /iiwa7/control_mode
llm_task_planner_node
  -> /llm_task/plan_json
llm_task_executor_node
  -> /sam3/prompt, sequential execution status
```

## Repository Layout

The repository is intentionally organized around the main runtime chain:

- `docs/`
  - Architecture source files. The main graph is `docs/workspace_architecture.dot`.
- `src/pybullet_ros2_sim/`
  - PyBullet robot simulation, RGB-D simulation, desired-joint generators, and the impedance controller.
- `src/perception_geometry/`
  - Mask and depth fusion utilities that extract a 3D keypoint from the camera stream.
- `src/sam3_ros/`
  - ROS 2 wrapper for SAM-based mask generation.
- `src/iiwa_state_udp_bridge/`
  - Bridge that republishes the simulation state into the compact `robot_states` stream used by the monitor.
- `src/robot_monitor/`
  - Qt monitor for joint position, velocity, torque, and desired values.
- `src/robot_control_msgs/`
  - Shared ROS messages.
- `src/iiwa_cpp_controller/`
  - Auxiliary C++ controller implementation. Not part of the default Python main chain.
- `src/udp_realtime_plot/`
  - Auxiliary plotting package. Not part of the default main chain.

Generated directories such as `build/`, `install/`, `log/`, `run_logs/`, and `run_pids/` are excluded from version control.

## Quick Start

### Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- `colcon`
- PyBullet and the Python dependencies required by the selected packages
- Qt runtime and development packages for `robot_monitor`

### Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Launch The Main Motion-Control Chain

Recommended minimal launcher:

```bash
./start_iiwa_stack_min.sh start
```

Simple launcher:

```bash
./start_iiwa_stack.sh
```

### Run The RGB-D And Perception Path

Recommended one-command launcher for the current tabletop demo:

```bash
./start_rekep_demo.sh start
```

Recommended one-command launcher for the LLM task-decomposition branch:

```bash
OPENAI_API_KEY=... ./start_llm_rekep_demo.sh start
```

This launcher starts:

- RGB-D simulation with the raised table and colored cubes
- `sam3_mask_node` inside `~/venvs/ros_vla`
- clustering-based keypoint fusion
- the keypoint tracker
- `robotstate_bridge`
- `robot_monitor`

The LLM launcher additionally starts:

- `llm_task_planner_node`
- `llm_task_executor_node`

Launch the RGB-D backend and perception nodes explicitly:

```bash
ros2 run pybullet_ros2_sim iiwa_pybullet_rgbd_sim_node
ros2 run perception_geometry mask_depth_fusion_node
ros2 run pybullet_ros2_sim iiwa_keypoint_tracker_node
```

Start SAM3 manually from its dedicated virtual environment:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_workspaces/humble/ros2_pybullet_ws/install/setup.bash
source ~/venvs/ros_vla/bin/activate

python -m sam3_ros.sam3_mask_node --ros-args \
  -p image_topic:=/sim/camera/color/image_raw \
  -p prompt:="red cube" \
  -p device:="cuda"
```

In the LLM branch, `sam3_mask_node` also listens on `/sam3/prompt`, so the
executor can switch target objects at runtime without restarting the node.

To send a natural-language task:

```bash
ros2 topic pub --once /llm_task/instruction std_msgs/msg/String "{data: '依次移动到红色方块、蓝色方块和黄色方块上方'}"
```

The RGB-D scene includes a raised table and multiple colored cubes so text prompts
such as `red cube` can target a specific object. The perception node publishes a
single primary keypoint for tracking plus clustered proposal points on
`/perception/keypoint_candidates`.

## Main Packages And Entry Points

- `pybullet_ros2_sim`
  - `iiwa_pybullet_sim_node`
  - `iiwa_pybullet_rgbd_sim_node`
  - `iiwa_line_ik_desired`
  - `iiwa_impedance_controller`
  - `iiwa_keypoint_tracker_node`
- `sam3_ros`
  - `sam3_mask_node`
- `perception_geometry`
  - `mask_depth_fusion_node`
- `iiwa_state_udp_bridge`
  - `robotstate_bridge`
- `robot_monitor`
  - `robot_monitor`

## Architecture Diagram

The maintained architecture source is:

- `docs/workspace_architecture.dot`

To render it locally:

```bash
mkdir -p docs/generated
dot -Tsvg docs/workspace_architecture.dot -o docs/generated/workspace_architecture.svg
```

## Notes

- Root launch scripts now resolve the workspace from the script location instead of assuming a hard-coded absolute path.
- `start_iiwa_monitoring_stack.sh` is an auxiliary template for custom trajectory-monitoring experiments rather than the default maintained launch path.
- This branch focuses on the cleaned main chain and removes older duplicate entry points that were not part of the maintained runtime path.
