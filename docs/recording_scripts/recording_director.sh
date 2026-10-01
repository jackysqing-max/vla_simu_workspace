#!/usr/bin/env bash
set -euo pipefail

WS="/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws"
SCRIPTS="$WS/docs/recording_scripts"

TASK="${TASK:-pick up the left silver surgical instrument and place it into the tray center}"
VISUAL_PROMPT="${VISUAL_PROMPT:-left silver surgical instrument}"
HOVER_TASK="${HOVER_TASK:-move above the left silver surgical instrument}"
CUBE_PROMPT="${CUBE_PROMPT:-red cube}"

cd "$WS/docs"

pause() {
  local message="$1"
  printf "\n%s\n" "$message"
  read -r -p "Press Enter to continue..." _
}

warn() {
  printf "[WARN] %s\n" "$*" >&2
}

run_allow_failure() {
  set +e
  "$@"
  local status=$?
  set -e
  if [[ "$status" -ne 0 ]]; then
    warn "command exited with status $status: $*"
    warn "If the GUI/windows are still useful, record the current state before continuing."
  fi
  return 0
}

header() {
  local number="$1"
  local title="$2"
  local gif_name="$3"
  printf "\n============================================================\n"
  printf "Director stage %s: %s\n" "$number" "$title"
  printf "Suggested GIF: %s\n" "$gif_name"
  printf "============================================================\n"
}

stop_stack() {
  "$SCRIPTS/00_stop_recording_stack.sh"
}

print_workspace_tip() {
  cat <<EOF

Workspace:
  cd $WS/docs

EOF
}

print_llm_topic_tips() {
  cat <<EOF

Optional topic terminals for this LLM clip:

  cd $WS
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 topic echo /llm_task/status

  cd $WS
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 topic echo /llm_task/plan_json

EOF
}

print_vision_topic_tips() {
  cat <<EOF

Optional topic terminals for the medical visual clip:

  cd $WS
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 topic echo /sam3/active_prompt

  cd $WS
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 topic echo /gms_keypoints/selected_keypoint_3d

  cd $WS
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 topic echo /gms_keypoints/valid

EOF
}

print_director_usage() {
  cat <<EOF
Usage:
  ./recording_scripts/recording_director.sh list
  ./recording_scripts/recording_director.sh all
  ./recording_scripts/recording_director.sh core
  ./recording_scripts/recording_director.sh vision
  ./recording_scripts/recording_director.sh sorting
  ./recording_scripts/recording_director.sh stop

Director stages:
  1  llm-schema       Text-only prompt, schema, multi-action JSON explanation
  2  llm-live         Live /llm_task/instruction -> /llm_task/plan_json
  3  vision           Medical scene SAM3 segmentation + GMS keypoint
  4  line             Cartesian straight-line robot trajectory
  5  circle           Cartesian circular robot trajectory
  6  sorting          Medical instrument grasp -> tray center -> release
  7  limitation       Keypoint jitter / identity switch limitation
  8  cube             Optional legacy cube-following demo

Recommended PPT core sequence:
  ./recording_scripts/recording_director.sh core

Environment overrides:
  TASK="$TASK"
  VISUAL_PROMPT="$VISUAL_PROMPT"
  HOVER_TASK="$HOVER_TASK"
  CUBE_PROMPT="$CUBE_PROMPT"
EOF
}

stage_1_llm_schema() {
  header "1" "LLM prompt / schema / multi-action JSON explanation" "01_llm_prompt_schema.gif"
  stop_stack
  print_workspace_tip
  pause "Start OBS on one large terminal. This stage prints the prompt design and JSON schema."
  "$SCRIPTS/01c_llm_prompt_schema_cheatsheet.sh"
  pause "Keep or stop OBS after the schema and prompt-handoff table are visible."
}

stage_2_llm_live() {
  header "2" "LLM live task input -> plan_json -> target_prompt handoff" "02_llm_plan_json_live.gif"
  stop_stack
  print_llm_topic_tips
  pause "Start OBS and optional topic terminals, then launch the LLM-only planner."
  "$SCRIPTS/01a_llm_prompt_stack_start.sh"
  pause "When /llm_task/status and /llm_task/plan_json windows are ready, send the task."
  "$SCRIPTS/01b_llm_prompt_send_task.sh" "$TASK"
  pause "Record the printed plan_json and target_prompt handoff, then continue to stop this stage."
  stop_stack
}

stage_3_medical_vision() {
  header "3" "Medical scene SAM3 segmentation + GMS keypoint" "03_medical_sam3_gms_keypoint.gif"
  stop_stack
  print_vision_topic_tips
  cat <<EOF

OBS windows to record:
  - SAM3 Mask
  - SAM3 Keypoint / overlay
  - optional PyBullet GUI showing the silver instruments and tray

This stage disables the point-cloud window and disables robot following.
Prompt:
  $VISUAL_PROMPT

EOF
  pause "Start OBS on the mask/keypoint windows, then launch the visual perception demo."
  run_allow_failure "$SCRIPTS/02a_medical_scene_visual_keypoint_recording.sh" "$VISUAL_PROMPT"
  pause "A valid keypoint has been requested. Record the stable mask and selected keypoint, then continue to stop."
  stop_stack
}

stage_4_line() {
  header "4" "Robot Cartesian straight-line trajectory" "04_robot_line_trajectory.gif"
  stop_stack
  pause "Start OBS on the PyBullet GUI, then launch the line trajectory demo."
  "$SCRIPTS/05_robot_line_trajectory_recording.sh"
  pause "Record at least one forward/back cycle, then continue to stop."
  stop_stack
}

stage_5_circle() {
  header "5" "Robot Cartesian circular trajectory" "05_robot_circle_trajectory_success.gif"
  stop_stack
  pause "Start OBS on the PyBullet GUI, then launch the verified circular trajectory demo."
  "$SCRIPTS/06_robot_circle_trajectory_recording.sh"
  pause "Record at least one complete circle, then continue to stop."
  stop_stack
}

stage_6_medical_sorting() {
  header "6" "Medical instrument sorting: grasp, move to tray center, release" "06_medical_instrument_sorting.gif"
  stop_stack
  pause "Start OBS on the PyBullet GUI, then launch the current sorting task."
  GRIPPER_MODEL="${GRIPPER_MODEL:-franka_hand}" \
    run_allow_failure "$SCRIPTS/09_medical_instrument_sorting_recording.sh" "$TASK"
  pause "Record grasp, lift, move above tray center, lower, and release, then continue to stop."
  stop_stack
}

stage_7_limitation() {
  header "7" "Known limitation: keypoint jitter / identity switch" "07_keypoint_jitter_limitation.gif"
  stop_stack
  pause "Start OBS on PyBullet plus keypoint overlay, then launch the limitation clip."
  run_allow_failure "$SCRIPTS/10_keypoint_jitter_limitation_recording.sh" "$HOVER_TASK"
  pause "Record keypoint jump or unstable following, then continue to stop."
  stop_stack
}

stage_8_cube_follow() {
  header "8" "Optional legacy cube following demo" "08_cube_following.gif"
  stop_stack
  pause "Start OBS on PyBullet plus keypoint overlay, then launch cube following."
  "$SCRIPTS/08_cube_following_recording.sh" "$CUBE_PROMPT"
  pause "Record cube tracking/following, then continue to stop."
  stop_stack
}

run_stage() {
  case "$1" in
    1|llm-schema) stage_1_llm_schema ;;
    2|llm-live) stage_2_llm_live ;;
    3|vision|medical-vision) stage_3_medical_vision ;;
    4|line) stage_4_line ;;
    5|circle) stage_5_circle ;;
    6|sorting|medical-sorting) stage_6_medical_sorting ;;
    7|limitation|jitter) stage_7_limitation ;;
    8|cube) stage_8_cube_follow ;;
    *)
      printf "Unknown director stage: %s\n" "$1" >&2
      exit 2
      ;;
  esac
}

main() {
  local arg="${1:-list}"
  case "$arg" in
    list|--list|-l)
      print_director_usage
      ;;
    stop)
      stop_stack
      ;;
    all)
      print_director_usage
      pause "This will run stages 1-8. Start/stop OBS manually at each prompt."
      for stage in 1 2 3 4 5 6 7 8; do
        run_stage "$stage"
      done
      printf "\nDirector sequence complete.\n"
      ;;
    core)
      print_director_usage
      pause "This will run the recommended PPT core sequence: LLM schema, LLM live, medical vision, circle, sorting, limitation."
      for stage in 1 2 3 5 6 7; do
        run_stage "$stage"
      done
      printf "\nCore director sequence complete.\n"
      ;;
    1|2|3|4|5|6|7|8|llm-schema|llm-live|vision|medical-vision|line|circle|sorting|medical-sorting|limitation|jitter|cube)
      run_stage "$arg"
      ;;
    *)
      print_director_usage
      exit 2
      ;;
  esac
}

main "$@"
