# Medical GMS/VLA Demo PPT Handover

Date: 2026-06-18

Workspace:

```bash
/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
```

PPT goal:

Use 2 slides to summarize the current medical instrument sorting demo and future work. The main story is:

```text
Natural-language instruction
  -> Qwen3 LLM task planning
  -> structured plan_json
  -> target_prompt handoff
  -> SAM3 segmentation
  -> GMS / RGB-D keypoint
  -> robot grasp, move to tray center, lower, and release
```


## 1. Recommended GIF List

Use 4 required GIFs plus 1 optional limitation GIF.

```text
01_llm_plan_json_live.gif
02_medical_sam3_gms_keypoint.gif
03_medical_instrument_sorting.gif
04_robot_circle_trajectory_success.gif
05_keypoint_jitter_limitation.gif
```

Priority:

```text
Must use:
  01_llm_plan_json_live.gif
  02_medical_sam3_gms_keypoint.gif
  03_medical_instrument_sorting.gif

Good support clip:
  04_robot_circle_trajectory_success.gif

Optional honest limitation:
  05_keypoint_jitter_limitation.gif
```

Do not overload the PPT with too many videos. For a 2-slide handover, the main sorting GIF should be visually largest.


## 2. Director Script For Recording

Run from:

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/docs
```

Recommended core recording flow:

```bash
./recording_scripts/recording_director.sh core
```

Only record medical visual segmentation and keypoint:

```bash
./recording_scripts/recording_director.sh vision
```

Only record medical sorting task:

```bash
./recording_scripts/recording_director.sh sorting
```

Stop all current recording/demo processes:

```bash
./recording_scripts/00_stop_recording_stack.sh
```


## 3. LLM Recording Clip

Current main prompt:

```text
pick up the left silver surgical instrument and place it into the tray center
```

Start Qwen3 planner:

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/docs

QWEN3_AUTO_START=true \
LLM_BACKEND=qwen3_local \
./recording_scripts/01a_llm_prompt_stack_start.sh
```

Topic monitor terminals:

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /llm_task/instruction
```

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /llm_task/status
```

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /llm_task/plan_json
```

Send task and pretty-print output:

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/docs

./recording_scripts/01b_llm_prompt_send_task.sh \
  "pick up the left silver surgical instrument and place it into the tray center"
```

Expected status:

```text
[STATUS] planning
[STATUS] planned: 3 steps
```

If it says:

```text
planned_fallback: 3 steps
```

then it used the deterministic fallback planner, not live Qwen3.

Expected plan structure:

```json
{
  "steps": [
    {
      "action": "grasp_target",
      "target_prompt": "left silver surgical instrument"
    },
    {
      "action": "hover_target",
      "target_prompt": "white sorting tray center"
    },
    {
      "action": "release_gripper",
      "target_prompt": ""
    }
  ]
}
```

PPT explanation:

```text
The user task is sent to /llm_task/instruction.
The planner wraps it with a system prompt, scene summary, and JSON schema.
Qwen3 returns a constrained plan_json.
Each target_prompt is used as the text interface to downstream perception.
```


## 4. Visual Segmentation And Keypoint Clip

Command:

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/docs

./recording_scripts/02a_medical_scene_visual_keypoint_recording.sh \
  "left silver surgical instrument"
```

Record these windows:

```text
SAM3 Mask
SAM3 Keypoint / overlay
Optional: PyBullet GUI showing silver instruments and tray
```

Point-cloud window is disabled by default.

Useful topic monitors:

```bash
ros2 topic echo /sam3/active_prompt
ros2 topic echo /gms_keypoints/selected_keypoint_3d
ros2 topic echo /gms_keypoints/valid
```

PPT explanation:

```text
The target_prompt from the LLM plan is reused as the SAM3 text prompt.
SAM3 segments the requested surgical instrument.
GMS / RGB-D processing selects a stable 3D keypoint and publishes it to the robot pipeline.
```


## 5. Main Robot Sorting Clip

Command:

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/docs

GRIPPER_MODEL=franka_hand \
./recording_scripts/09_medical_instrument_sorting_recording.sh \
  "pick up the left silver surgical instrument and place it into the tray center"
```

Record:

```text
Approach target instrument
Close gripper
Lift
Move above tray center
Lower near tray
Release gripper
```

PPT explanation:

```text
This is the integrated demo: language planning, visual grounding, keypoint-based grasp target, and robot execution.
The task order is now constrained so release happens only after the tray-center target is selected and reached.
```


## 6. Motion Baseline Clip

Circular Cartesian trajectory:

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/docs

./recording_scripts/06_robot_circle_trajectory_recording.sh
```

PPT explanation:

```text
This clip shows baseline Cartesian motion capability and validates the low-level trajectory stack separately from the LLM/vision task pipeline.
```


## 7. Limitation Clip

Command:

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/docs

./recording_scripts/10_keypoint_jitter_limitation_recording.sh \
  "move above the left silver surgical instrument"
```

PPT explanation:

```text
Current limitation: similar silver instruments can cause keypoint identity switch or jitter.
This can destabilize following and motivates persistent object identity tracking.
```


## 8. Two-Slide PPT Structure

### Slide 1: Current System Pipeline

Title:

```text
LLM-guided Medical Instrument Sorting Pipeline
```

Layout:

```text
Left side:
  Pipeline diagram:

  User instruction
    -> Qwen3 planner
    -> plan_json
    -> target_prompt
    -> SAM3 mask
    -> GMS / RGB-D keypoint
    -> robot executor
    -> grasp and place

Right side:
  GIF 1: LLM plan_json live output
  GIF 2: SAM3 mask + GMS keypoint
```

Slide 1 talking points:

```text
1. Qwen3 converts natural language into a constrained JSON action sequence.
2. plan_json contains action primitives: grasp_target, hover_target, release_gripper.
3. target_prompt is the interface from language planning to visual perception.
4. SAM3 and GMS ground each text target into a 3D keypoint.
```


### Slide 2: Demo Result And Next Work

Title:

```text
Integrated Demo Result And Future Work
```

Layout:

```text
Large center/left GIF:
  Medical instrument sorting demo

Small support GIF:
  Cartesian circular trajectory

Bottom/right text:
  Current limitations and next steps
```

Slide 2 talking points:

```text
Current results:
  - Multi-step task planning from a single English instruction.
  - Prompt-driven segmentation and 3D keypoint grounding.
  - Robot executes grasp, lift, move to tray center, lower, and release.
  - Circular Cartesian trajectory demo validates the motion control stack.

Current limitations:
  - Keypoint can jump between similar silver instruments.
  - Grasp is visually acceptable but not fully physically robust.
  - Tray center detection still needs more robust geometric grounding.

Next work:
  - Persistent object identity across prompt switches and occlusion.
  - Better grasp pose estimation using object principal axis and plane normal.
  - Closed-loop grasp confirmation before lifting.
  - Collision-aware tray placement with stable tray center estimation.
  - Extend from single instrument sorting to multi-instrument sequential tasks.
```


## 9. Suggested Narration

One-sentence summary:

```text
The system converts an open natural-language medical sorting instruction into a constrained robot action plan, grounds each target through text-driven segmentation and 3D keypoint extraction, and executes the grasp-and-place sequence in simulation.
```

Short Chinese version:

```text
当前系统将自然语言任务通过 Qwen3 拆解为结构化 plan_json，再用 target_prompt 驱动 SAM3 和 GMS 完成视觉关键点定位，最后由机械臂执行抓取、移动到托盘中心和释放。
```


## 10. Windows PPT Notes

Recommended workflow:

```text
1. Record clips with OBS on Ubuntu.
2. Convert videos to GIF or MP4.
3. Put all exported files into one Windows folder:
   ppt_assets/
4. Keep GIF filenames simple:
   01_llm_plan_json_live.gif
   02_medical_sam3_gms_keypoint.gif
   03_medical_instrument_sorting.gif
   04_robot_circle_trajectory_success.gif
   05_keypoint_jitter_limitation.gif
5. Insert the GIFs into PPT from the same folder.
```

For PPT, MP4 is often smoother and smaller than GIF. If GIF file size is too large, use MP4 in PowerPoint and set it to play automatically.
