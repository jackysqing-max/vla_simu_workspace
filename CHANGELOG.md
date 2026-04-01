# Changelog

## v0.1.3 - 2026-04-01

- Added `start_semantic_tracking_demo.sh` as the stable semantic-tracking entrypoint for the tabletop RGB-D scene.
- Added `semantic_prompt_cli.py` so semantic targets such as `red cube` and `blue cube` can be switched directly from the terminal.
- Added `clean_demo_processes.sh` and a matching `clean` subcommand to simplify resetting demo processes and ROS state.
- Updated `mask_depth_fusion_node` to estimate the visible top-surface center of each cube in the world frame instead of using a centroid-like target.
- Updated `iiwa_keypoint_tracker_node` and `llm_task_executor_node` so hover tracking can be enabled and disabled explicitly and uses a shared hover-offset target above the object surface.
- Updated `llm_task_planner_node` and `task_plan_utils` with a local fallback planner so simple ordered color instructions still produce executable steps when the OpenAI request fails.
- Reworked the README and launcher flow to separate the stable semantic demo from the experimental LLM task-decomposition demo.

## v0.1.2 - 2026-03-25

- Added `llm_task_planner_node` to decompose natural-language instructions into a validated JSON step plan using the OpenAI Responses API with structured outputs.
- Added `llm_task_executor_node` to execute plan steps sequentially by switching `/sam3/prompt` targets and waiting for the end effector to reach each hover goal.
- Updated `sam3_mask_node` so the segmentation prompt can be changed at runtime through `/sam3/prompt`.
- Added `start_llm_rekep_demo.sh` as a one-command launcher for the RGB-D tabletop scene, SAM3 virtual environment, clustering perception, tracker, monitor, and LLM task pipeline.

## v0.1.1 - 2026-03-25

- Added a visible raised table to the RGB-D PyBullet scene instead of a buried table URDF.
- Added multiple tabletop cubes with different colors so text prompts such as `red cube` select a specific object.
- Replaced the single-centroid depth fusion logic with a ReKep-style clustering proposal stage:
  - masked RGB-D sampling
  - PCA feature compression
  - k-means grouping
  - 3D mean-shift proposal merging
- Kept the existing `/perception/keypoint_3d` topic for tracker compatibility and added `/perception/keypoint_candidates` for clustered proposals.
- Added `start_rekep_demo.sh` as a one-command launcher for:
  - RGB-D simulation
  - SAM3 mask generation in `~/venvs/ros_vla`
  - clustering-based keypoint fusion
  - keypoint tracker
  - state bridge
  - robot monitor
