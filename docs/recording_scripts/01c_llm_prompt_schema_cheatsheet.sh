#!/usr/bin/env bash
set -euo pipefail

cat <<'EOF'
================ LLM TASK PLANNING CLIP ================

Goal:
  Show prompt input, constrained task decomposition, plan_json, and prompt handoff.
  This is a text-only LLM recording clip. No PyBullet, SAM3, GMS, or GUI is needed.


================ INPUT TASK ================

pick up the left silver surgical instrument and place it into the tray center


================ PLANNER ROLE ================

The LLM is used as a high-level task planner.
It does not output robot joint commands.
It must output a constrained JSON plan that the robot executor can consume.

Allowed primitive actions:
  1. grasp_target
     - close the parallel gripper on one perceived object
     - requires target_prompt

  2. hover_target
     - move above one perceived object or placement target
     - requires target_prompt

  3. release_gripper
     - open the gripper at the current placement pose
     - target_prompt must be empty

  4. wait
     - pause for a short duration
     - target_prompt must be empty


================ MULTI-ACTION RULE ================

For pick-and-place / sorting tasks, the plan order must be:

  grasp_target(object)
  hover_target(destination tray center)
  release_gripper()

This prevents the old failure mode:

  grasp_target(object)
  release_gripper()
  hover_target(tray)

The release must happen only after the destination prompt has been grounded and reached.


================ TARGET_PROMPT DESIGN ================

target_prompt is the bridge from language planning to perception.
It should be a short text phrase that SAM3/fusion can ground visually.

Good examples:
  left silver surgical instrument
  right silver surgical instrument
  white sorting tray center
  gauze pad
  curved needle
  red entry point

Bad examples:
  pick up the object
  put it there
  complete the task


================ PLAN JSON SCHEMA ================

{
  "task_summary": "string",
  "planning_notes": "string",
  "steps": [
    {
      "step_index": 1,
      "action": "hover_target | grasp_target | release_gripper | wait",
      "target_prompt": "string",
      "description": "string",
      "success_radius_m": 0.0,
      "dwell_sec": 0.0,
      "wait_sec": 0.0
    }
  ]
}


================ EXPECTED PLAN JSON ================

{
  "task_summary": "Pick up the left silver surgical instrument and place it into the tray center.",
  "planning_notes": "Use the object prompt for grasping, then switch to the tray-center prompt before releasing.",
  "steps": [
    {
      "step_index": 1,
      "action": "grasp_target",
      "target_prompt": "left silver surgical instrument",
      "description": "grasp the left silver surgical instrument",
      "success_radius_m": 0.0,
      "dwell_sec": 1.0,
      "wait_sec": 0.0
    },
    {
      "step_index": 2,
      "action": "hover_target",
      "target_prompt": "white sorting tray center",
      "description": "move above the center of the sorting tray",
      "success_radius_m": 0.0,
      "dwell_sec": 0.8,
      "wait_sec": 0.0
    },
    {
      "step_index": 3,
      "action": "release_gripper",
      "target_prompt": "",
      "description": "release the gripper inside the tray",
      "success_radius_m": 0.0,
      "dwell_sec": 0.0,
      "wait_sec": 0.8
    }
  ]
}


================ PROMPT HANDOFF ================

step 1:
  action: grasp_target
  target_prompt -> SAM3/fusion:
    left silver surgical instrument

step 2:
  action: hover_target
  target_prompt -> SAM3/fusion:
    white sorting tray center

step 3:
  action: release_gripper
  no new visual prompt
  executor opens the gripper after reaching the placement pose


================ EXECUTION CONTRACT ================

The executor must treat each grounded target as a stable waypoint:

  1. Send target_prompt to perception.
  2. Wait until the keypoint or tray center is stable.
  3. Lock that target position for the current action.
  4. Finish the grasp / hover / release primitive.
  5. Only then advance to the next plan step.

For static recording scenes, locking the grounded keypoint after stability avoids
keypoint jumping to another instrument after the robot starts moving.

EOF
