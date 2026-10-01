# PPT video storyboard

## Recommended order

1. **Robot Motion Baseline: Straight Line**
   - Script: `05_robot_line_trajectory_recording.sh`
   - OBS target: PyBullet GUI
   - Message: the iiwa simulation/control stack can execute a repeatable Cartesian line trajectory.

2. **Robot Motion Baseline: Circular Trajectory**
   - Script: `06_robot_circle_trajectory_recording.sh`
   - OBS target: PyBullet GUI
   - Message: the same `/iiwa7/joint_desired` interface can drive a circular Cartesian path.

3. **SAM3 Prompt Segmentation**
   - Script: `07_sam3_prompt_segmentation_recording.sh "red cube"`
   - OBS targets: `SAM3 Mask`, `SAM3 Keypoint`, `Masked 3D Point Cloud`
   - Message: prompt-driven segmentation grounds language targets in RGB-D observations.

4. **Cube Following**
   - Script: `08_cube_following_recording.sh "red cube"`
   - OBS targets: PyBullet GUI + `SAM3 Keypoint`
   - Message: SAM3 + RGB-D keypoint can drive closed-loop hover tracking over a selected cube.

5. **LLM Task Decomposition and Prompt Handoff**
   - Scripts: `01a_llm_prompt_stack_start.sh`, then `01b_llm_prompt_send_task.sh`
   - OBS targets: terminal status, `run_logs/planner.log`, optional `/llm_task/plan_json`
   - Message: Qwen3 converts a natural-language task into structured JSON and a compact visual prompt.

6. **Medical Instrument Sorting**
   - Script: `09_medical_instrument_sorting_recording.sh`
   - OBS targets: PyBullet GUI, optionally terminal status
   - Message: the current VLA-style medical demo connects language, prompt segmentation, GMS keypoints, and robot actions.

7. **Known Limitation: Keypoint Jitter**
   - Script: `10_keypoint_jitter_limitation_recording.sh`
   - OBS targets: PyBullet GUI + `SAM3 Keypoint`
   - Message: the latest GMS path still has candidate identity jumps, so following can be unstable.

## Suggested PPT grouping

### Current work

- Robot control baseline: straight / circular motion.
- Visual grounding baseline: SAM3 prompt segmentation.
- Visual servoing baseline: cube following.
- Integrated medical scenario: task decomposition, prompt handoff, keypoint following, grasp/release.

### Next work

- Add memory-based GMS candidate selector.
- Stabilize object plane normal/tangent and align tangent sign across frames.
- Freeze keypoint updates during approach/closing/holding.
- Define close/lift/place success predicates and use a named `grasp_attachment_approximation` if PyBullet contact is too noisy.

## Suggested GIF names

```text
01_robot_line_trajectory.gif
02_robot_circle_trajectory.gif
03_sam3_prompt_segmentation.gif
04_cube_following.gif
05_llm_task_prompt_handoff.gif
06_medical_instrument_sorting.gif
07_keypoint_jitter_limitation.gif
```
