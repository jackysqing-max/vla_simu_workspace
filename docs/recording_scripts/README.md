# OBS recording stage scripts

Run these from the repository root or from this directory. Use `00_stop_recording_stack.sh`
between stages so old ROS nodes, Qwen3, SAM3, and PyBullet processes do not overlap.

## Guided OBS sequence

Recommended director script for PPT recording:

```bash
docs/recording_scripts/recording_director.sh core
```

Run every clip in order with prompts:

```bash
docs/recording_scripts/obs_recording_sequence.sh
```

Run one stage only:

```bash
docs/recording_scripts/obs_recording_sequence.sh 4
```

## 0. Stop current stack

```bash
docs/recording_scripts/00_stop_recording_stack.sh
```

## 1. LLM task decomposition and prompt handoff

This clip is text-only. It shows natural-language input, the constrained JSON
task plan, and how each `target_prompt` would be handed to perception later.
It does not start PyBullet, SAM3, GMS, or any visualizer.

Static prompt/schema explanation:

```bash
docs/recording_scripts/01c_llm_prompt_schema_cheatsheet.sh
```

Live ROS topic input/output:

```bash
docs/recording_scripts/01a_llm_prompt_stack_start.sh
```

Then open optional topic terminals and start OBS:

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /llm_task/status
ros2 topic echo /llm_task/plan_json
```

Send the task with:

```bash
docs/recording_scripts/01b_llm_prompt_send_task.sh \
  "pick up the left silver surgical instrument and place it into the tray center"
```

One-shot mode:

```bash
docs/recording_scripts/01_llm_task_prompt_recording.sh \
  "pick up the left silver surgical instrument and place it into the tray center"
```

OBS target:

- terminal output from the script
- optional ROS topic echo windows for `/llm_task/status` and `/llm_task/plan_json`
- the `target_prompt` handoff table printed by `01b`

By default, `01a` uses an already-running Qwen3 service if available. If Qwen3
is not running, it falls back to the deterministic local planner so recording
does not block. For a live Qwen3 clip:

```bash
QWEN3_AUTO_START=true LLM_BACKEND=qwen3_local \
  docs/recording_scripts/01a_llm_prompt_stack_start.sh
```

## 2. Visual perception: SAM3 + GMS

```bash
docs/recording_scripts/02a_medical_scene_visual_keypoint_recording.sh \
  "left silver surgical instrument"
```

OBS targets:

- enlarged `SAM3 Mask`
- enlarged `SAM3 Keypoint`
- point-cloud window is disabled

## 3. Robot hover/follow motion

```bash
docs/recording_scripts/03_robot_hover_follow_recording.sh \
  "move above the left silver surgical instrument"
```

OBS targets:

- PyBullet GUI
- `SAM3 Keypoint` if you also want the keypoint tracking evidence

## 4. Robot grasp/place action

```bash
docs/recording_scripts/04_robot_grasp_place_recording.sh \
  "pick up the left silver surgical instrument and then release it"
```

OBS target:

- PyBullet GUI

## Suggested GIF names

```text
01_llm_task_decomposition.gif
02_prompt_to_sam3.gif
03_sam3_mask.gif
04_gms_keypoint_overlay.gif
05_masked_3d_point_cloud.gif
06_robot_hover_follow.gif
07_robot_grasp_place.gif
```

## Additional standalone clips

Baseline robot motion:

```bash
docs/recording_scripts/05_robot_line_trajectory_recording.sh
docs/recording_scripts/06_robot_circle_trajectory_recording.sh
```

Standalone SAM3 segmentation:

```bash
docs/recording_scripts/07_sam3_prompt_segmentation_recording.sh "red cube"
docs/recording_scripts/07_sam3_prompt_segmentation_recording.sh "blue cube"
```

Cube following:

```bash
docs/recording_scripts/08_cube_following_recording.sh "red cube"
```

Medical instrument sorting:

```bash
docs/recording_scripts/09_medical_instrument_sorting_recording.sh \
  "pick up the left silver surgical instrument and place it into the tray center"
```

Known limitation / keypoint jitter:

```bash
docs/recording_scripts/10_keypoint_jitter_limitation_recording.sh \
  "move above the left silver surgical instrument"
```
