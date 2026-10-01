#!/usr/bin/env bash
set -euo pipefail

WS="/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws"
PROMPT="${*:-red cube}"
LOG_DIR="$WS/run_logs"
PID_DIR="$WS/run_pids"
cd "$WS"
mkdir -p "$LOG_DIR" "$PID_DIR"

# OBS targets: PyBullet GUI, SAM3 Mask, SAM3 Keypoint. This records the older
# cube-following chain where SAM3 + RGB-D keypoint drives iiwa hover tracking.
SIM_GUI="${SIM_GUI:-true}" \
SAM3_PROMPT="$PROMPT" \
TRACKER_START_ENABLED=true \
TRACKER_MAX_JOINT_STEP_RAD="${TRACKER_MAX_JOINT_STEP_RAD:-0.06}" \
HOVER_OFFSET_Z="${HOVER_OFFSET_Z:-0.12}" \
./start_semantic_tracking_demo.sh start

setsid bash -lc "
  set +u
  source /opt/ros/humble/setup.bash
  source '$WS/install/setup.bash'
  set -u 2>/dev/null || true
  exec ros2 run vla_rgbd_tools tracking_overlay_viewer_node --ros-args \
    -p color_topic:=/sim/camera/color/image_raw \
    -p depth_topic:=/sim/camera/aligned_depth_to_color/image_raw \
    -p camera_info_topic:=/sim/camera/color/camera_info \
    -p mask_topic:=/sam3/mask \
    -p points_topic:=/perception/masked_points \
    -p overlay_topic:=/perception/keypoint_overlay \
    -p prompt_topic:=/sam3/prompt \
    -p score_topic:=/sam3/score \
    -p keypoint_topic:=/perception/keypoint_3d \
    -p keypoint_px_topic:=/perception/keypoint_px \
    -p valid_topic:=/perception/valid \
    -p display_scale:=2.2 \
    -p status_panel_width:=380 \
    -p prefer_overlay_image:=false
" >"$LOG_DIR/cube_follow_viewer.log" 2>&1 &
echo "$!" > "$PID_DIR/cube_follow_viewer.pid"
echo "[OK] cube following recording stack started with prompt: $PROMPT"
