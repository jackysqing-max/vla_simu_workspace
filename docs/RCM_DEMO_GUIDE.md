# RCM Demo Guide

This guide covers the maintained surgical remote-center-of-motion (RCM) paths:

1. controller-only virtual fixtures;
2. SAM3 + RGB-D port perception;
3. the complete Qwen3/SAM3/RCM task chain.

All commands assume the workspace root as the current directory.

> The system is a research simulation, not a certified medical controller.

## Design boundary

The language model may select only these four actions, in this order:

1. `localize_rcm_port`
2. `align_tool_axis`
3. `establish_rcm`
4. `execute_rcm_circle`

The deterministic safety layer supplies target prompts, tolerances, insertion
depth, circle radius, dwell time, and action ordering. The LLM never publishes
joint angles, torques, poses, or unconstrained free-space motion.

```text
natural-language instruction
  -> llm_task_planner_node
  -> surgical_rcm_task_executor_node
       -> SAM3 prompt -> vlm_port_pose_node
       -> locked port center and axis
       -> rcm_virtual_fixture_node
       -> /iiwa7/joint_desired
       -> PyBullet iiwa
```

The controller crosses the phantom surface only along the locked port axis:

```text
PREINSERT_HOLD -> APPROACH_PORT -> PORT_DWELL
-> INSERT_THROUGH_PORT -> INSERTED_DWELL -> RCM_PIVOT
```

Each transition requires both a minimum dwell and a position-tolerance check.

## Build

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select \
  robot_control_msgs \
  pybullet_ros2_sim \
  perception_geometry \
  sam3_ros \
  rcm_virtual_fixtures \
  --symlink-install
source install/setup.bash
```

The dVRK Large Needle Driver visual meshes, license, source notice, and URDF
are under:

```text
src/rcm_virtual_fixtures/meshes/dvrk_lnd_420006/
src/rcm_virtual_fixtures/urdf/dvrk_lnd_420006_tip.urdf
```

The visual meshes have no collision geometry and do not alter the RCM
controller. The phantom's canonical mesh is:

```text
src/rcm_virtual_fixtures/meshes/phantom_centered.stl
```

## Controller-only virtual fixtures

Choose `rcm`, `line`, or `plane`:

```bash
RCM_FIXTURE_MODE=rcm ./start_rcm_virtual_fixture_demo.sh start
RCM_FIXTURE_MODE=line ./start_rcm_virtual_fixture_demo.sh start
RCM_FIXTURE_MODE=plane ./start_rcm_virtual_fixture_demo.sh start
./start_rcm_virtual_fixture_demo.sh stop
```

Frequently tuned variables:

```bash
RCM_SPEED_MPS=0.018
RCM_TRAJECTORY_RADIUS_M=0.020
RCM_TRAJECTORY_LENGTH_M=0.100
RCM_TOOL_LENGTH_M=0.220
RCM_MAX_JOINT_STEP_RAD=0.012
RCM_SAFE_INSERTION=true
RCM_SHOW_PORT_OVERLAY=true
RCM_GUI=true
```

## Port-perception demo

```bash
./start_vlm_rcm_port_perception_demo.sh start
./start_vlm_rcm_port_perception_demo.sh stop
```

SAM3 uses the prompt `circular hole`. RGB-D estimates the three-dimensional
opening center and locks it after multiple stable frames. The current phantom's
hidden channel is not observable from a single exterior view, so the inward
axis comes from calibrated CAD geometry; status output reports
`axis_source=calibrated_port_geometry` explicitly.

Main outputs:

- `/vlm_rcm/overlay`
- `/vlm_rcm/status`
- `/vlm_rcm/locked_port_point`
- `/vlm_rcm/locked_port_axis`
- `/vlm_rcm/port_ready`

## Complete LLM + VLM + RCM demo

The default launcher uses staged GPU scheduling: Qwen3 creates the constrained
plan, then releases GPU memory before SAM3 and the motion stack start.

```bash
./start_llm_vlm_rcm_demo.sh start
./start_llm_vlm_rcm_demo.sh task
./start_llm_vlm_rcm_demo.sh status
./start_llm_vlm_rcm_demo.sh stop
```

Send an explicit instruction when needed:

```bash
./start_llm_vlm_rcm_demo.sh task \
  "locate the circular port, align the tool, establish RCM, then execute a circle"
```

Validate without a Qwen3 service:

```bash
LLM_BACKEND=local_fallback ./start_llm_vlm_rcm_demo.sh start
./start_llm_vlm_rcm_demo.sh task
```

Portable environment overrides:

```bash
QWEN3_VENV=/path/to/qwen3-vllm
SAM3_VENV=/path/to/sam3-environment
QWEN_MODEL=Qwen/Qwen3-4B
QWEN3_PORT=8000
GPU_EXECUTION_MODE=staged_single_gpu
```

Key motion values:

```bash
RCM_PREINSERT_CLEARANCE_M=0.040
RCM_SPEED_MPS=0.018
RCM_TRAJECTORY_RADIUS_M=0.020
RCM_TOOL_LENGTH_M=0.220
RCM_LAMBDA=0.650
```

Main interfaces:

- `/llm_task/instruction`: natural-language task input
- `/llm_task/plan_json`: fixed-schema plan
- `/sam3/prompt`: active perception target
- `/surgical_rcm/status`: executor progress
- `/surgical_rcm/active_step_json`: active safe action
- `/rcm_virtual_fixtures/stage`: low-level motion stage
- `/rcm_virtual_fixtures/metrics`: tracking and RCM errors
- `/iiwa7/joint_desired`: controller output

Only `rcm_virtual_fixture_node` publishes `/iiwa7/joint_desired` in the complete
RCM chain. The executor performs sequencing and gating; it does not publish
joint commands.

## Recording and metrics

Run one fixture at a time:

```bash
./docs/recording_scripts/11_rcm_virtual_fixture_recording.sh rcm
./docs/recording_scripts/11_rcm_virtual_fixture_recording.sh line
./docs/recording_scripts/11_rcm_virtual_fixture_recording.sh plane
```

Runtime logs and CSV metrics are written under `run_logs/`. That directory is
ignored and must not be committed.

Useful checks:

```bash
ros2 topic echo /surgical_rcm/status
ros2 topic echo /rcm_virtual_fixtures/stage
ros2 topic echo /vlm_rcm/status
ros2 topic hz /iiwa7/joint_desired
```

## Validation

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  pybullet_ros2_sim rcm_virtual_fixtures \
  --symlink-install
colcon test --packages-select \
  pybullet_ros2_sim rcm_virtual_fixtures
colcon test-result --verbose

source install/setup.bash
ros2 launch rcm_virtual_fixtures \
  rcm_virtual_fixture_demo.launch.py --show-args
```

Check shell syntax without starting a demo:

```bash
bash -n \
  start_llm_vlm_rcm_demo.sh \
  start_rcm_virtual_fixture_demo.sh \
  start_vlm_rcm_port_perception_demo.sh \
  docs/recording_scripts/11_rcm_virtual_fixture_recording.sh
```

## Troubleshooting

- **Qwen3 does not start:** set `QWEN3_VENV` and inspect
  `run_logs/llm_vlm_rcm_qwen3.log`.
- **SAM3 runs out of memory:** keep
  `GPU_EXECUTION_MODE=staged_single_gpu` or use a CPU device.
- **Port never locks:** inspect `/vlm_rcm/overlay`, SAM3 score, depth validity,
  and the configured stable-frame thresholds.
- **Motion never advances:** compare `/rcm_virtual_fixtures/stage` with the
  configured dwell and position tolerance.
- **No joint commands:** verify that the executor accepted a four-step plan and
  that `/vlm_rcm/port_ready` became true.

Detailed phantom geometry is documented in
[PHANTOM_3D_MODELING_SPEC.txt](PHANTOM_3D_MODELING_SPEC.txt).
