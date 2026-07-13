# RCM Virtual Fixture Demo

This project package adapts the old Franka RCM / virtual fixture demos from:

`/home/siqin/old_ws/ysq_proj/rcm_decouple/src/franka_pkg/src/franka_task.zip`

The original code used libfranka robot callbacks and Panda torque / velocity interfaces. The new package keeps the task-space demo meaning, but connects to the current PyBullet KUKA iiwa simulation interface:

- input: `/iiwa7/joint_states`
- output: `/iiwa7/joint_desired`
- mode: `/iiwa7/control_mode = 1` position mode

## Modes

- `rcm`: fixed remote-center-of-motion point. A virtual tool tip moves around a small circle while the shaft line passes through the fixed RCM point.
- `line`: straight-line virtual fixture. The end effector moves along one constrained line.
- `plane`: planar virtual fixture. The end effector moves inside a constrained plane.

The PyBullet view includes a silver surgical shaft and a visual-only dVRK
Large Needle Driver 420006 wrist/jaw assembly rigidly aligned with the KUKA
link-7 `+Z` axis. Its geometric tip uses the same
`tool_length_m` value as the RCM controller:

- silver rod: surgical-tool shaft
- articulated silver jaws: dVRK Large Needle Driver 420006 distal assembly
- cyan ring: locked port center used by the insertion controller
- cyan arrow: locked inward port axis
- red sphere: fixed RCM point
- yellow sphere: desired tool-tip point
- green points: actual tool-tip trajectory

The rod and dVRK jaw assembly are visual-only and have no collision geometry,
so they do not disturb the existing RCM controller or introduce artificial
contact forces. Source meshes, license, URDF, and preview image are stored in:

```text
src/rcm_virtual_fixtures/meshes/dvrk_lnd_420006/
src/rcm_virtual_fixtures/urdf/dvrk_lnd_420006_tip.urdf
```

RCM mode starts outside the port and executes:

```text
PREINSERT_HOLD -> APPROACH_PORT -> PORT_DWELL
-> INSERT_THROUGH_PORT -> INSERTED_DWELL -> RCM_PIVOT
```

Crossing the phantom surface is constrained to the fixed port axis. A stage
does not advance until its minimum duration has elapsed and the actual tip has
remained inside the configured position tolerance.

The `rcm` mode also loads `phantom_centered.stl` by default. The model is
scaled from millimeters to meters and positioned so the middle of its
30 mm top opening coincides with the fixed RCM point:

- STL bounds: `320 x 260 x 166 mm`
- STL local origin: bottom center
- top opening: centered at local `(0, 0, 164) mm`
- PyBullet base position: `(0.701726, 0.0, 0.240701) m`
- visual alpha: `0.50`
- collision: disabled by default for recording performance
- table center: `(0.9, 0.0) m`
- table top: `z=0.240701 m`, coincident with the phantom bottom
- RCM marker radius: `4 mm`, leaving the 30 mm opening visible

## Build

From the workspace root:

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rcm_virtual_fixtures --symlink-install
source install/setup.bash
```

## OBS Recording Commands

Run one mode at a time:

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/docs

./recording_scripts/11_rcm_virtual_fixture_recording.sh rcm
./recording_scripts/11_rcm_virtual_fixture_recording.sh line
./recording_scripts/11_rcm_virtual_fixture_recording.sh plane
```

Stop the stack:

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
./start_rcm_virtual_fixture_demo.sh stop
```

## Direct Startup Commands

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws

RCM_FIXTURE_MODE=rcm ./start_rcm_virtual_fixture_demo.sh start
RCM_FIXTURE_MODE=line ./start_rcm_virtual_fixture_demo.sh start
RCM_FIXTURE_MODE=plane ./start_rcm_virtual_fixture_demo.sh start
```

Useful tuning variables:

```bash
RCM_SPEED_MPS=0.012
RCM_TRAJECTORY_RADIUS_M=0.018
RCM_TRAJECTORY_LENGTH_M=0.080
RCM_MAX_JOINT_STEP_RAD=0.012
RCM_TOOL_LENGTH_M=0.220
RCM_TOOL_RADIUS_M=0.006
RCM_CAMERA_DISTANCE_M=0.90
RCM_CAMERA_YAW_DEG=42.0
RCM_CAMERA_PITCH_DEG=-38.0
RCM_PHANTOM_ALPHA=0.50
RCM_TRACK_FORCE=260.0
RCM_TRACK_POS_GAIN=0.65
RCM_TRACK_MAX_VEL=6.0
RCM_MARKER_RADIUS_M=0.004
RCM_TIP_MARKER_RADIUS_M=0.002
RCM_TRACE_POINT_RADIUS_M=0.0025
RCM_TRACE_MIN_INSERTED_DEPTH_M=0.065
RCM_SHOW_PORT_OVERLAY=true
RCM_PORT_RING_RADIUS_M=0.010
RCM_PORT_AXIS_OUTSIDE_M=0.050
RCM_PORT_AXIS_INSIDE_M=0.090
RCM_SAFE_INSERTION=true
RCM_PREINSERT_CLEARANCE_M=0.040
RCM_PORT_STANDOFF_M=0.012
RCM_INSERTION_SPEED_MPS=0.018
RCM_TABLE_X_M=0.9
RCM_TABLE_TOP_Z_M=0.240701
```

Phantom controls:

```bash
RCM_SHOW_PHANTOM=false
RCM_PHANTOM_COLLISION=false
RCM_PHANTOM_ALPHA=0.30
RCM_PHANTOM_X_M=0.701726
RCM_PHANTOM_Y_M=0.0
RCM_PHANTOM_Z_M=0.240701
```

For a longer and more visible tool:

```bash
RCM_TOOL_LENGTH_M=0.280 \
RCM_TOOL_RADIUS_M=0.008 \
RCM_FIXTURE_MODE=rcm \
./start_rcm_virtual_fixture_demo.sh start
```

`RCM_TOOL_LENGTH_M` is passed to both the RCM controller and the rendered
tool. Keep this shared value so the red RCM marker remains on the visible
shaft and the yellow TIP marker matches the physical tool end.

## Topics To Show

```bash
ros2 topic echo /rcm_virtual_fixtures/status
ros2 topic echo /rcm_virtual_fixtures/metrics
ros2 topic echo /rcm_virtual_fixtures/rcm_point
ros2 topic echo /rcm_virtual_fixtures/tip_point
ros2 topic echo /rcm_virtual_fixtures/locked_port_point
ros2 topic echo /rcm_virtual_fixtures/locked_port_axis
```

`/rcm_virtual_fixtures/metrics` publishes:

1. elapsed time
2. RCM error
3. tool-tip trajectory tracking error
4. plane error
5. line error
6. target end-effector x
7. target end-effector y
8. target end-effector z
9. end-effector trajectory tracking error
10. RCM radial error (distance from fixed RCM point to actual shaft axis)
11. signed RCM axial error

## RCM Error Recording

The `rcm` startup mode records actual-state errors to a timestamped CSV by
default:

```bash
/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/run_logs/rcm_metrics_YYYYMMDD_HHMMSS.csv
```

The latest recording is also available through:

```bash
/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/run_logs/rcm_metrics_latest.csv
```

The CSV contains the fixed RCM point error, point-to-shaft radial error,
signed axial error, KUKA end-effector trajectory error, tool-tip trajectory
error, and all corresponding desired/actual Cartesian positions.

Disable recording or select a custom output path with:

```bash
RCM_RECORD_METRICS=false ./start_rcm_virtual_fixture_demo.sh start

RCM_METRICS_CSV=/tmp/my_rcm_run.csv \
./start_rcm_virtual_fixture_demo.sh start
```

## PPT Framing

Recommended clips:

1. RCM virtual fixture: show the virtual tool direction pivoting around a fixed remote center.
2. Straight-line fixture: show constrained Cartesian line motion.
3. Plane fixture: show planar constrained motion.
4. Monitor/status terminal: show `rcm_err`, `line_err`, or `plane_err` as quantitative evidence.

This is a kinematic PyBullet KUKA adaptation for demonstration and recording. It is not a direct torque-level replacement for the old Franka controller.
