# ROS 2 PyBullet IIWA Workspace

ROS 2 Humble workspace for KUKA iiwa simulation, perception-guided motion,
LLM task planning, surgical remote-center-of-motion (RCM) constraints, and
touchless gesture control.

Current release: `v1.3.0`

> This is a research simulation. It is not a certified medical control system.

## Runtime paths

The repository maintains four focused paths:

```text
Motion control
  PyBullet iiwa -> joint states -> desired joints -> impedance control

Semantic tracking
  RGB-D camera -> SAM3 mask -> depth fusion -> 3D target -> iiwa tracker

Surgical RCM
  instruction -> Qwen3/fallback planner -> SAM3 port pose
              -> safe align/insert -> constrained circular motion

Touchless control
  camera landmarks -> gesture command -> MRI viewer or iiwa end-effector target
```

The LLM selects only a constrained task sequence. Deterministic code owns
geometry, motion parameters, safety ordering, and joint commands.

## Repository layout

- `src/pybullet_ros2_sim/`: iiwa simulation, RGB-D camera, planners, trackers,
  and Python impedance control.
- `src/rcm_virtual_fixtures/`: RCM controller, port-pose estimator, surgical
  executor, launch file, URDF, and licensed dVRK visual meshes.
- `src/touchless_project_ros/`: maintained ROS touchless gesture package.
- `src/perception_geometry/`: mask/depth fusion and 3D target extraction.
- `src/sam3_ros/`: SAM3 ROS wrapper.
- `src/vla_rgbd_tools/`: RealSense launch and visualization helpers.
- `src/iiwa_cpp_controller/`: optional C++ controller and gesture mapping.
- `src/iiwa_state_udp_bridge/`, `src/robot_monitor/`: state bridge and Qt
  monitor.
- `src/robot_control_msgs/`: shared ROS interfaces.
- `docs/`: architecture, deployment, teaching, and release notes.

Generated output (`build/`, `install/`, `log/`, `run_logs/`, and `run_pids/`)
is not versioned.

## Requirements

- Ubuntu 22.04 and ROS 2 Humble
- Python 3.10, `colcon`, and PyBullet
- Qt development/runtime packages when building `robot_monitor`
- Optional: CUDA, SAM3, Qwen3/vLLM, and RealSense dependencies for their
  respective demos

Install declared ROS dependencies and build:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Quick start

Run commands from the workspace root.

### Minimal iiwa stack

```bash
./start_iiwa_stack_min.sh start
./start_iiwa_stack_min.sh stop
```

### Semantic RGB-D tracking

```bash
./start_semantic_tracking_demo.sh start
./start_semantic_tracking_demo.sh prompt "blue cube"
./start_semantic_tracking_demo.sh stop
```

The compatibility entry point `./start_rekep_demo.sh start` starts the same
maintained path.

### LLM task planning

Use the OpenAI-compatible backend configured by the launcher:

```bash
OPENAI_API_KEY=... ./start_llm_rekep_demo.sh start
./start_llm_rekep_demo.sh task \
  "依次移动到红色方块、蓝色方块和黄色方块上方"
./start_llm_rekep_demo.sh stop
```

For a local Qwen3 service:

```bash
LLM_BACKEND=qwen3_local \
GPU_EXECUTION_MODE=staged_single_gpu \
./start_llm_rekep_demo.sh start
```

### Surgical RCM demos

Run the controller-only virtual fixture:

```bash
RCM_FIXTURE_MODE=rcm ./start_rcm_virtual_fixture_demo.sh start
./start_rcm_virtual_fixture_demo.sh stop
```

Run port perception only:

```bash
./start_vlm_rcm_port_perception_demo.sh start
./start_vlm_rcm_port_perception_demo.sh stop
```

Run the complete Qwen3 + SAM3 + RCM chain:

```bash
./start_llm_vlm_rcm_demo.sh start
./start_llm_vlm_rcm_demo.sh task
./start_llm_vlm_rcm_demo.sh status
./start_llm_vlm_rcm_demo.sh stop
```

If Qwen3 is unavailable, validate the rest of the chain with the fixed safe
planner:

```bash
LLM_BACKEND=local_fallback ./start_llm_vlm_rcm_demo.sh start
```

Set `QWEN3_VENV` or `SAM3_VENV` when the environments are not in the launchers'
portable default locations. See [the RCM guide](docs/RCM_DEMO_GUIDE.md) for
interfaces, parameters, and troubleshooting.

### Touchless gesture control

Run gesture-to-robot control:

```bash
ros2 launch touchless_project touchless_robot_demo.launch.py
```

The MRI viewer intentionally does not bundle medical volumes. Provide a local
NIfTI file explicitly:

```bash
TOUCHLESS_PROJECT_NIFTI_PATH=/private/path/volume.nii \
  ros2 launch touchless_project touchless_demo.launch.py
```

Medical images, papers, experiment results, and Office documents must remain in
private storage and must not be committed to this public repository.

## Useful interfaces

- `/iiwa7/joint_states`, `/iiwa7/joint_desired`, `/iiwa7/joint_torques`
- `/sim/camera/*`
- `/sam3/prompt`, `/sam3/mask`, `/sam3/score`
- `/perception/keypoint_3d`
- `/llm_task/instruction`, `/llm_task/plan_json`
- `/vlm_rcm/locked_port_point`, `/vlm_rcm/locked_port_axis`
- `/rcm_virtual_fixtures/stage`, `/rcm_virtual_fixtures/metrics`
- `/surgical_rcm/status`
- `/touchless/command_json`

## Documentation

- [RCM demo guide](docs/RCM_DEMO_GUIDE.md)
- [v1.3.0 cleanup summary](docs/v1.3.0_cleanup_summary.md)
- [Qwen3 local deployment](docs/qwen3_local_deployment.md)
- [Workspace architecture source](docs/workspace_architecture.dot)
- [Teaching materials](docs/teaching/)

Render the architecture diagram locally:

```bash
mkdir -p docs/generated
dot -Tsvg docs/workspace_architecture.dot \
  -o docs/generated/workspace_architecture.svg
```

## Validation

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

Release history is preserved in Git tags and `release/*` branches. The exact
pre-cleanup snapshot for this release is preserved on
`backup/pre-cleanup-20260713`.
