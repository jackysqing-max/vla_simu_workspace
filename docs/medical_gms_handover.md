# Medical GMS/VLA Demo Handover

Date: 2026-06-16
Workspace: `/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws`

This file is intended as a handover note for starting a new chat without losing the current project context. In this project, "GMS" means the Give-me-scissors-style keypoint workflow.

## Current Goal

Build a staged ROS2/PyBullet VLA-style demo for a medical tabletop sorting task:

- Use an LLM planner to decompose a natural-language instruction.
- Pass compact visual prompts and execution steps through JSON.
- Use SAM3 for open-vocabulary segmentation.
- Use RGB-D geometry to generate GMS-style candidate keypoints, object plane normal, and tangent axis.
- Use KUKA iiwa with an attached parallel gripper, currently Franka hand in the GMS keypoint demo, to hover/follow/grasp/place surgical instruments.
- Keep the architecture staged for single-GPU memory limits: run Qwen3 planning first, stop it, then run SAM3 perception/execution.

The current near-term task is not to polish the full paper. It is to make the medical PyBullet demo technically coherent: stable keypoints, stable hover tracking, grasp-state logic, and a credible medical sorting scene.

## Important Launch Scripts

Main GMS keypoint/following demo:

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
./start_give_me_scissors_keypoint_demo.sh
```

Task-style medical grasp demo:

```bash
cd /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws
./start_medical_grasp_demo.sh task "pick up the left silver surgical instrument and then release it"
```

Stop the general stack:

```bash
./start_llm_rekep_demo.sh stop
```

Useful diagnostics:

```bash
source install/setup.bash
python3 scripts/diagnose_keypoint_jitter.py --duration-sec 25 --prompt "left silver surgical instrument"
ros2 topic echo /gms_keypoints/selected_keypoint_3d --once
ros2 topic echo /gms_keypoints/object_plane_normal --once
ros2 topic echo /iiwa7/keypoint_tracker_status --once
```

## Current Main Files

- `start_give_me_scissors_keypoint_demo.sh`
  - Starts the GMS keypoint debug/following configuration.
  - Defaults to `SIM_SCENE_PRESET=medical_grasp`.
  - Defaults to `GRIPPER_MODEL=franka_hand`.
  - Enables PyBullet keypoint overlay.
  - Uses `/gms_keypoints/*` topics for selected keypoint and plane frame.

- `start_medical_grasp_demo.sh`
  - Starts task-level grasp/release demo.
  - Defaults to `GRIPPER_MODEL=kuka_parallel`, but GMS demo currently uses `franka_hand`.
  - Includes grasp-contact parameters, pregrasp/lift/release timing, and tray parameters.

- `src/perception_geometry/perception_geometry/give_me_scissors_keypoint_node.py`
  - Generates GMS-style RGB-D candidate keypoints from SAM3 mask.
  - Publishes:
    - `/gms_keypoints/selected_keypoint_3d`
    - `/gms_keypoints/selected_keypoint_px`
    - `/gms_keypoints/object_plane_normal`
    - `/gms_keypoints/object_plane_tangent`
    - `/gms_keypoints/metadata_json`
    - `/gms_keypoints/overlay`
  - Already has a selected-keypoint stabilizer, but raw candidate identities still jump.

- `src/perception_geometry/perception_geometry/mask_depth_fusion_node.py`
  - Older/open-vocab mask-depth fusion path.
  - Publishes `/perception/keypoint_3d`, `/perception/object_plane_normal`, `/perception/object_plane_tangent`.

- `src/pybullet_ros2_sim/pybullet_ros2_sim/iiwa_keypoint_tracker_node.py`
  - Tracks a perceived keypoint and publishes iiwa joint targets.
  - Can use object plane normal/tangent to align the gripper.
  - Current GMS demo uses `/gms_keypoints/selected_keypoint_3d` plus plane topics.

- `src/pybullet_ros2_sim/pybullet_ros2_sim/iiwa_pybullet_rgbd_sim_node.py`
  - PyBullet scene, gripper, grasp simulation, medical objects, white tray, and overlays.
  - Contains Franka hand attachment code, grasp contact/attachment logic, and medical scene spawning.

- `scripts/diagnose_keypoint_jitter.py`
  - New diagnostic probe for mask/keypoint/axis jitter.
  - Subscribes to SAM3 mask/score and GMS keypoint/plane/metadata topics.
  - Reports mask IoU, centroid jumps, candidate count, selected index changes, raw candidate jumps, stable keypoint jumps, and normal/tangent angle changes.

## Current Scene State

The medical scene is based on `scene_preset:=medical_grasp`.

Current features:

- Blue split surgical instrument mesh assets from `assets/surgical_instruments_4729067/files/split/`.
- Only two split blue objects are currently loaded; the third component is commented out in `_spawn_medical_grasp_asset_objects`.
- White sorting tray is added on the table.
- External scene object interfaces exist:
  - `EXTERNAL_SCENE_OBJECTS_JSON`
  - `EXTERNAL_SCENE_OBJECTS_FILE`
- Current default GMS keypoint prompt:
  - `SAM3_PROMPT="left silver surgical instrument"`
- The user wants a medical sorting direction: separate visible surgical instruments and eventually place an instrument into the tray.

Important caveat:

- Some surgical meshes have difficult collision geometry for real physical grasping.
- A surrogate grasp attachment mode may be needed for stable demo behavior, but it should be clearly named as an approximation, not presented as real force-closure grasping.

## What Has Already Been Debugged

The latest isolated debug was run by manually starting only the necessary nodes:

1. PyBullet sim, no GUI, medical scene, Franka hand.
2. SAM3 on CUDA.
3. Mask-depth fusion.
4. GMS keypoint node.
5. Optional hover follower.
6. `scripts/diagnose_keypoint_jitter.py` for measurement.

Static visual chain result, without hover follower:

- SAM3 mask was stable:
  - mean mask IoU = 1.0
  - mask centroid jump = 0 px
  - score approximately 0.720
  - mask area approximately 138 px
- GMS final selected keypoint was stable:
  - stable selected keypoint p95 approximately 0.54 mm
  - raw selected p95 approximately 4.55 mm
- Object plane was stable:
  - normal max angle approximately 0.003 deg
  - tangent max angle approximately 0.0005 deg
  - no sign flips

With hover follower enabled:

- SAM3 mask was still stable enough:
  - mean IoU approximately 0.991
  - centroid p95 approximately 0.10 px
  - score approximately 0.65
  - mask area approximately 140-144 px
- Raw candidate selection sometimes jumped:
  - raw selected candidate max jump approximately 17 mm
  - selected index changed between nearby candidates
- Stabilized selected output was much calmer:
  - stable selected keypoint p95 approximately 1.83 mm
  - normal p95 approximately 0.62 deg
  - tangent p95 approximately 0.17 deg
  - no sign flips

Conclusion:

The object is not actually moving, and SAM3 is not the main problem. The visible jump comes from per-frame GMS candidate generation and candidate identity changes. Raw candidates and candidate indices are not temporally stable. The final `/gms_keypoints/selected_keypoint_3d` is already stabilized, but overlays/metadata can still show raw candidate jitter.

## Current Root Cause Hypothesis

GMS-style candidate generation recomputes candidates every frame:

1. Mask pixels are converted to RGB-D points.
2. Features are built from XYZ, RGB, and UV.
3. PCA + kmeans produce several candidate clusters.
4. Cluster representative points are merged and sorted by pixel position.
5. A selected candidate is chosen.

Even with a fixed kmeans seed, tiny mask/depth/view changes can change:

- cluster membership,
- representative point,
- candidate ordering,
- selected index,
- raw candidate position.

Original GMS/ReKep-style systems often allow a VLM/LLM to select from a candidate set for a task. In this ROS2/PyBullet architecture, if the winner is selected every frame without persistent identity, the "best" candidate can jump even while the object is static.

## Proposed Next Fix

Implement a memory-based candidate selector, not just a low-pass filter.

Recommended logic:

```text
TRACKING:
  generate candidates each frame
  score candidates with semantic + geometry + mask + reachability terms
  prefer candidate nearest to current locked keypoint
  update current keypoint only if candidate is inside lock_radius
  switch only if new score exceeds current score by margin AND distance < switch_radius
  if all candidates invalid for N frames, enter REACQUIRE

APPROACHING / GRASPING:
  freeze selected keypoint and object plane frame
  do not reselect grasp point while the gripper/arm occludes the object

HOLDING:
  stop using visual keypoint for grasp target updates
  track held object through gripper-relative pose or grasp attachment state

PLACING / RELEASING:
  use planned target/tray pose, not a constantly reselected object keypoint
```

Concrete implementation targets:

- In `give_me_scissors_keypoint_node.py`:
  - Add candidate scores to metadata.
  - Add `current_locked_keypoint_xyz`, `current_score`, `lock_radius`, `switch_radius`, `score_margin`.
  - Choose nearest candidate to current locked point first.
  - Do not publish raw selected index as the meaningful stable identity.
  - Add stabilizers for plane normal/tangent, not just point position.
  - Align tangent sign to the previous frame to avoid PCA sign ambiguity.

- In `iiwa_keypoint_tracker_node.py` or task executor:
  - Add explicit state topic or parameter to freeze keypoint updates during `APPROACHING`, `CLOSING`, and `HOLDING`.
  - Once grasp is closed/held, do not keep driving based on new visual keypoints.

- In PyBullet overlay:
  - Show only stable selected keypoint by default.
  - Hide raw candidate cloud/indices unless in debug mode.

## Grasp Success Discussion

Current PyBullet code has several grasp-related mechanisms:

- It checks fingertip contact counts against a target body.
- It can require actual physical contact.
- It can attach the nearest graspable body after contact/near-center conditions.
- It reports gripper status strings such as no contact, waiting contact, object held, contact lost.

For a stable research demo, define grasp success with a staged predicate:

```text
close_success =
  gripper command is closed
  AND fingertip center / grasp frame is near selected keypoint
  AND left/right fingertip contacts exist OR surrogate keypoint attachment is active

lift_success =
  close_success
  AND object bottom or object center z increased by threshold
  AND object remains close to gripper frame for a hold duration

place_success =
  held object is released
  AND object is inside tray footprint
  AND object is resting/stable
```

If PyBullet contact remains unstable for thin surgical tools, use a named approximation:

```text
grasp_attachment_approximation:
  when fingertip midpoint reaches selected keypoint and gripper closes,
  store object-to-gripper relative pose,
  maintain that transform during lift/place,
  release by removing the constraint/attachment.
```

Do not describe this as real force-closure. It is a controllable VLA execution abstraction for a prototype.

## LLM Prompt Passing / Innovation Framing

The current architecture:

```text
Natural language task
  -> Qwen3 planner
  -> JSON plan with target_prompt and actions
  -> stop Qwen3 to free GPU
  -> SAM3 prompt-based segmentation
  -> RGB-D keypoint/plane extraction
  -> ROS2 execution
```

The idea of using an LLM to create/modify visual prompts is feasible, but by itself is probably not a strong novelty claim because related works already combine LLM/VLM perception and robot execution.

Better framing:

- A staged, resource-aware VLA-style ROS2/PyBullet pipeline for medical tabletop manipulation.
- LLM-generated visual prompts and action JSON as the interface between language planning and SAM3 perception.
- GMS-style candidate keypoints adapted to a modular ROS2 system.
- Temporal candidate memory and grasp-state freezing to make open-vocabulary keypoints executable, not just visible.
- A medical sorting scenario with surgical instruments and tray placement.

This is stronger than claiming "LLM changes prompt" as the core contribution.

## Current Known Problems

- Raw GMS candidate points/indices can jump frame to frame.
- The stable selected keypoint is calmer, but visual overlays may still make the system appear unstable.
- Current plane normal/tangent are not fully temporally filtered.
- The GMS metadata prompt may appear empty because active prompt publication is not latched/timed as expected; this was observed but not proven to affect keypoint stability.
- Grasping thin surgical meshes with PyBullet contact is not yet physically convincing.
- After grasp/hold begins, visual keypoint updates should be frozen, but this logic still needs to be made explicit.
- There are many modified/untracked files in the working tree; do not reset or revert unrelated changes.

## Suggested Next Chat Opening Prompt

Use this prompt in the new chat:

```text
请先阅读 /home/siqin/ros2_workspaces/humble/ros2_pybullet_ws/docs/medical_gms_handover.md。
我现在要继续改当前 ROS2/PyBullet 医疗 GMS/VLA demo。重点是：
1. 给 GMS 候选关键点加入带记忆的分数门控和切换阈值，避免候选点跳变；
2. 对 object plane normal/tangent 做时间稳定和符号对齐；
3. 在 APPROACHING/CLOSING/HOLDING 阶段冻结 keypoint，不再边抓边重新选择；
4. 明确抓取成功判定，必要时实现命名为 grasp_attachment_approximation 的近似抓取。
请先检查相关代码再改，不要重置我的其他改动。
```

## Verification Already Done

The diagnostic script compiles:

```bash
python3 -m py_compile scripts/diagnose_keypoint_jitter.py
```

Manual isolated ROS2 runs were stopped afterward. `pgrep` showed no remaining processes matching:

```text
iiwa_pybullet_rgbd_sim_node
give_me_scissors_keypoint_node
mask_depth_fusion_node
sam3_mask_node
iiwa_keypoint_tracker_node
```
