#!/usr/bin/env bash
set -euo pipefail

WS="/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws"
SCRIPTS="$WS/docs/recording_scripts"

TASK="${TASK:-pick up the left silver surgical instrument and then release it}"
VISUAL_TASK="${VISUAL_TASK:-locate the left silver surgical instrument}"
HOVER_TASK="${HOVER_TASK:-move above the left silver surgical instrument}"
SAM3_PROMPT="${SAM3_PROMPT:-left silver surgical instrument}"
CUBE_PROMPT="${CUBE_PROMPT:-red cube}"

cd "$WS/docs"

pause() {
  local message="$1"
  printf "\n%s\n" "$message"
  read -r -p "Press Enter to continue..." _
}

stop_stack() {
  "$SCRIPTS/00_stop_recording_stack.sh"
}

print_header() {
  local number="$1"
  local title="$2"
  local gif_name="$3"
  printf "\n============================================================\n"
  printf "Stage %s: %s\n" "$number" "$title"
  printf "Suggested GIF: %s\n" "$gif_name"
  printf "============================================================\n"
}

print_llm_topic_tips() {
  cat <<EOF

Optional topic windows for this LLM clip:

  cd $WS
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 topic echo /llm_task/status
  ros2 topic echo /llm_task/plan_json
  ros2 topic echo /sam3/active_prompt

EOF
}

stage_1_llm() {
  print_header "1" "LLM task decomposition and prompt handoff" "01_llm_task_decomposition.gif"
  stop_stack
  pause "Start OBS recording, then start the LLM/prompt stack."

  ENABLE_GRIPPER="${ENABLE_GRIPPER:-true}" \
  GRIPPER_MODEL="${GRIPPER_MODEL:-franka_hand}" \
  "$SCRIPTS/01a_llm_prompt_stack_start.sh"

  print_llm_topic_tips
  pause "When the stack is ready and OBS is recording, send the LLM task."

  "$SCRIPTS/01b_llm_prompt_send_task.sh" "$TASK"

  pause "Keep OBS recording until plan_json and SAM3 prompt handoff are visible."
  stop_stack
}

stage_2_visual() {
  print_header "2" "Visual perception: SAM3 + GMS keypoint" "02_sam3_gms_keypoint.gif"
  stop_stack
  pause "Start OBS recording on SAM3 Mask / Keypoint / Point Cloud windows, then launch this stage."

  "$SCRIPTS/02_visual_sam3_gms_recording.sh" "$VISUAL_TASK"

  pause "Record the mask, selected keypoint, and masked point cloud."
  stop_stack
}

stage_3_line() {
  print_header "3" "Robot Cartesian straight-line trajectory" "03_robot_line_trajectory.gif"
  stop_stack
  pause "Start OBS recording on the PyBullet GUI, then launch the line trajectory."

  "$SCRIPTS/05_robot_line_trajectory_recording.sh"

  pause "Record at least one forward/back straight-line cycle."
  stop_stack
}

stage_4_circle() {
  print_header "4" "Robot Cartesian circular trajectory" "04_robot_circle_trajectory_success.gif"
  stop_stack
  pause "Start OBS recording on the PyBullet GUI, then launch the verified circle trajectory."

  "$SCRIPTS/06_robot_circle_trajectory_recording.sh"

  pause "Record at least one full circle. Default is position mode for a clean Cartesian demo."
  stop_stack
}

stage_5_sam3_prompt() {
  print_header "5" "Standalone SAM3 prompt segmentation" "05_sam3_prompt_segmentation.gif"
  stop_stack
  pause "Start OBS recording on the SAM3 windows, then launch prompt segmentation."

  "$SCRIPTS/07_sam3_prompt_segmentation_recording.sh" "$SAM3_PROMPT"

  pause "Record the prompt-driven mask and keypoint overlay."
  stop_stack
}

stage_6_cube_follow() {
  print_header "6" "Cube following demo" "06_cube_following.gif"
  stop_stack
  pause "Start OBS recording on PyBullet + keypoint overlay, then launch cube following."

  "$SCRIPTS/08_cube_following_recording.sh" "$CUBE_PROMPT"

  pause "Record the cube keypoint and iiwa hover/follow behavior."
  stop_stack
}

stage_7_medical_sorting() {
  print_header "7" "Medical instrument grasp / sorting task" "07_medical_instrument_sorting.gif"
  stop_stack
  pause "Start OBS recording on the PyBullet GUI, then launch the grasp/release task."

  GRIPPER_MODEL="${GRIPPER_MODEL:-franka_hand}" \
  "$SCRIPTS/09_medical_instrument_sorting_recording.sh" "$TASK"

  pause "Record approach, grasp, lift, and release."
  stop_stack
}

stage_8_limitation() {
  print_header "8" "Known limitation: keypoint jitter during following" "08_keypoint_jitter_limitation.gif"
  stop_stack
  pause "Start OBS recording on PyBullet + keypoint overlay, then launch the limitation clip."

  "$SCRIPTS/10_keypoint_jitter_limitation_recording.sh" "$HOVER_TASK"

  pause "Record the keypoint jump and the resulting unstable hover/follow behavior."
  stop_stack
}

run_stage() {
  case "$1" in
    1) stage_1_llm ;;
    2) stage_2_visual ;;
    3) stage_3_line ;;
    4) stage_4_circle ;;
    5) stage_5_sam3_prompt ;;
    6) stage_6_cube_follow ;;
    7) stage_7_medical_sorting ;;
    8) stage_8_limitation ;;
    *)
      printf "Unknown stage: %s\n" "$1" >&2
      exit 2
      ;;
  esac
}

list_stages() {
  cat <<EOF
Usage:
  ./recording_scripts/obs_recording_sequence.sh        # run all stages interactively
  ./recording_scripts/obs_recording_sequence.sh list   # list stages
  ./recording_scripts/obs_recording_sequence.sh 4      # run one stage
  ./recording_scripts/obs_recording_sequence.sh stop   # stop current stack

Stages:
  1  LLM task decomposition and prompt handoff
  2  Visual perception: SAM3 + GMS keypoint
  3  Robot Cartesian straight-line trajectory
  4  Robot Cartesian circular trajectory
  5  Standalone SAM3 prompt segmentation
  6  Cube following demo
  7  Medical instrument grasp / sorting task
  8  Known limitation: keypoint jitter during following

Environment overrides:
  TASK="$TASK"
  VISUAL_TASK="$VISUAL_TASK"
  HOVER_TASK="$HOVER_TASK"
  SAM3_PROMPT="$SAM3_PROMPT"
  CUBE_PROMPT="$CUBE_PROMPT"
EOF
}

main() {
  local arg="${1:-all}"
  case "$arg" in
    list|--list|-l)
      list_stages
      ;;
    stop)
      stop_stack
      ;;
    all)
      list_stages
      pause "This will run stages 1-8. Start OBS manually for each stage when prompted."
      for stage in 1 2 3 4 5 6 7 8; do
        run_stage "$stage"
        if [[ "$stage" != "8" ]]; then
          pause "Stage $stage finished. Prepare the next OBS scene."
        fi
      done
      printf "\nAll recording stages finished.\n"
      ;;
    [1-8])
      run_stage "$arg"
      ;;
    *)
      list_stages
      exit 2
      ;;
  esac
}

main "$@"
