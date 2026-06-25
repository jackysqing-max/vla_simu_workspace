from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    color_topic = LaunchConfiguration("color_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    start_realsense = LaunchConfiguration("start_realsense")
    realsense_camera_name = LaunchConfiguration("realsense_camera_name")
    realsense_camera_namespace = LaunchConfiguration("realsense_camera_namespace")
    realsense_serial_no = LaunchConfiguration("realsense_serial_no")
    realsense_color_profile = LaunchConfiguration("realsense_color_profile")
    realsense_depth_profile = LaunchConfiguration("realsense_depth_profile")
    realsense_log_level = LaunchConfiguration("realsense_log_level")
    prompt = LaunchConfiguration("prompt")
    prompt_topic = LaunchConfiguration("prompt_topic")
    active_prompt_topic = LaunchConfiguration("active_prompt_topic")
    sam3_python = LaunchConfiguration("sam3_python")
    sam3_device = LaunchConfiguration("sam3_device")
    sam3_infer_hz = LaunchConfiguration("sam3_infer_hz")
    sam3_max_side = LaunchConfiguration("sam3_max_side")
    sam3_score_th = LaunchConfiguration("sam3_score_th")
    sam3_mask_th = LaunchConfiguration("sam3_mask_th")
    target_frame = LaunchConfiguration("target_frame")
    depth_scale = LaunchConfiguration("depth_scale")
    min_score = LaunchConfiguration("min_score")
    max_frame_age_sec = LaunchConfiguration("max_frame_age_sec")
    cluster_count = LaunchConfiguration("cluster_count")
    cluster_max_samples = LaunchConfiguration("cluster_max_samples")
    cluster_meanshift_bandwidth_m = LaunchConfiguration("cluster_meanshift_bandwidth_m")
    use_top_surface_estimator = LaunchConfiguration("use_top_surface_estimator")
    top_surface_band_m = LaunchConfiguration("top_surface_band_m")
    top_surface_min_fraction = LaunchConfiguration("top_surface_min_fraction")
    top_surface_percentile = LaunchConfiguration("top_surface_percentile")
    keypoint_filter_alpha = LaunchConfiguration("keypoint_filter_alpha")
    keypoint_jump_reset_m = LaunchConfiguration("keypoint_jump_reset_m")
    keypoint_jump_hold_frames = LaunchConfiguration("keypoint_jump_hold_frames")
    candidate_lock_radius_m = LaunchConfiguration("candidate_lock_radius_m")
    invalid_reset_frames = LaunchConfiguration("invalid_reset_frames")
    prompt_settle_sec = LaunchConfiguration("prompt_settle_sec")
    enable_container_center_keypoint = LaunchConfiguration("enable_container_center_keypoint")
    container_center_inner_fraction = LaunchConfiguration("container_center_inner_fraction")
    container_center_z_percentile = LaunchConfiguration("container_center_z_percentile")
    container_center_z_offset_m = LaunchConfiguration("container_center_z_offset_m")
    container_center_min_points = LaunchConfiguration("container_center_min_points")
    container_center_expected_world = LaunchConfiguration("container_center_expected_world")
    container_center_expected_max_distance_m = LaunchConfiguration(
        "container_center_expected_max_distance_m"
    )
    overlay_topic = LaunchConfiguration("overlay_topic")
    start_overlay_viewer = LaunchConfiguration("start_overlay_viewer")
    start_robot_monitor = LaunchConfiguration("start_robot_monitor")
    monitor_keypoint_timeout_sec = LaunchConfiguration("monitor_keypoint_timeout_sec")
    viewer_window_name = LaunchConfiguration("viewer_window_name")
    viewer_refresh_hz = LaunchConfiguration("viewer_refresh_hz")
    viewer_cloud_stride = LaunchConfiguration("viewer_cloud_stride")
    viewer_cloud_display_max_points = LaunchConfiguration("viewer_cloud_display_max_points")

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_realsense", default_value="true"),
            DeclareLaunchArgument("realsense_camera_name", default_value="camera"),
            DeclareLaunchArgument("realsense_camera_namespace", default_value="camera"),
            DeclareLaunchArgument(
                "realsense_serial_no",
                default_value="",
                description=(
                    "Optional RealSense serial number. Raw digits are accepted and "
                    "are forced to remain a string when passed to the camera node."
                ),
            ),
            DeclareLaunchArgument("realsense_color_profile", default_value="848x480x30"),
            DeclareLaunchArgument("realsense_depth_profile", default_value="848x480x30"),
            DeclareLaunchArgument("realsense_log_level", default_value="info"),
            DeclareLaunchArgument(
                "color_topic",
                default_value="/camera/camera/color/image_raw",
            ),
            DeclareLaunchArgument(
                "depth_topic",
                default_value="/camera/camera/aligned_depth_to_color/image_raw",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/camera/color/camera_info",
            ),
            DeclareLaunchArgument("prompt", default_value="red cube"),
            DeclareLaunchArgument("prompt_topic", default_value="/sam3/prompt"),
            DeclareLaunchArgument("active_prompt_topic", default_value="/sam3/active_prompt"),
            DeclareLaunchArgument(
                "sam3_python",
                default_value=PathJoinSubstitution(
                    [EnvironmentVariable("HOME"), "venvs", "ros_vla", "bin", "python"]
                ),
                description="Python executable used to launch sam3_mask_node.",
            ),
            DeclareLaunchArgument("sam3_device", default_value="cuda"),
            DeclareLaunchArgument("sam3_infer_hz", default_value="2.0"),
            DeclareLaunchArgument("sam3_max_side", default_value="640"),
            DeclareLaunchArgument("sam3_score_th", default_value="0.10"),
            DeclareLaunchArgument("sam3_mask_th", default_value="0.40"),
            DeclareLaunchArgument("overlay_topic", default_value="/perception/keypoint_overlay"),
            DeclareLaunchArgument("start_overlay_viewer", default_value="true"),
            DeclareLaunchArgument("start_robot_monitor", default_value="true"),
            DeclareLaunchArgument(
                "viewer_window_name",
                default_value="SAM3 Real-Time Tracking",
            ),
            DeclareLaunchArgument("viewer_refresh_hz", default_value="20.0"),
            DeclareLaunchArgument(
                "target_frame",
                default_value="camera_color_optical_frame",
                description=(
                    "Frame used for top-surface estimation and scene registry. "
                    "Set this to the camera optical frame if no external TF is available."
                ),
            ),
            DeclareLaunchArgument(
                "depth_scale",
                default_value="0.001",
                description="Scale applied to uint16 depth frames to convert them to meters.",
            ),
            DeclareLaunchArgument("min_score", default_value="0.05"),
            DeclareLaunchArgument("max_frame_age_sec", default_value="0.2"),
            DeclareLaunchArgument("cluster_count", default_value="4"),
            DeclareLaunchArgument("cluster_max_samples", default_value="512"),
            DeclareLaunchArgument("cluster_meanshift_bandwidth_m", default_value="0.04"),
            DeclareLaunchArgument("use_top_surface_estimator", default_value="false"),
            DeclareLaunchArgument("top_surface_band_m", default_value="0.012"),
            DeclareLaunchArgument("top_surface_min_fraction", default_value="0.15"),
            DeclareLaunchArgument("top_surface_percentile", default_value="92.0"),
            DeclareLaunchArgument("keypoint_filter_alpha", default_value="0.18"),
            DeclareLaunchArgument("keypoint_jump_reset_m", default_value="0.08"),
            DeclareLaunchArgument("keypoint_jump_hold_frames", default_value="12"),
            DeclareLaunchArgument("candidate_lock_radius_m", default_value="0.08"),
            DeclareLaunchArgument("invalid_reset_frames", default_value="30"),
            DeclareLaunchArgument("prompt_settle_sec", default_value="0.8"),
            DeclareLaunchArgument("enable_container_center_keypoint", default_value="true"),
            DeclareLaunchArgument("container_center_inner_fraction", default_value="0.55"),
            DeclareLaunchArgument("container_center_z_percentile", default_value="20.0"),
            DeclareLaunchArgument("container_center_z_offset_m", default_value="0.0"),
            DeclareLaunchArgument("container_center_min_points", default_value="48"),
            DeclareLaunchArgument("container_center_expected_world", default_value="[0.0,0.0,0.0]"),
            DeclareLaunchArgument("container_center_expected_max_distance_m", default_value="0.0"),
            DeclareLaunchArgument("monitor_keypoint_timeout_sec", default_value="1.0"),
            DeclareLaunchArgument("viewer_cloud_stride", default_value="2"),
            DeclareLaunchArgument("viewer_cloud_display_max_points", default_value="8000"),
            Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                namespace=realsense_camera_namespace,
                name=realsense_camera_name,
                emulate_tty=True,
                output="screen",
                condition=IfCondition(start_realsense),
                arguments=["--ros-args", "--log-level", realsense_log_level],
                parameters=[
                    {
                        "serial_no": ParameterValue(realsense_serial_no, value_type=str),
                        "enable_color": True,
                        "enable_depth": True,
                        "enable_sync": True,
                        "align_depth.enable": True,
                        "pointcloud.enable": False,
                        "rgb_camera.color_profile": ParameterValue(
                            realsense_color_profile,
                            value_type=str,
                        ),
                        "depth_module.depth_profile": ParameterValue(
                            realsense_depth_profile,
                            value_type=str,
                        ),
                    }
                ],
            ),
            ExecuteProcess(
                name="sam3_mask_node",
                output="screen",
                cmd=[
                    sam3_python,
                    "-m",
                    "sam3_ros.sam3_mask_node",
                    "--ros-args",
                    "-r",
                    "__node:=sam3_mask_node",
                    "-p",
                    ["image_topic:=", color_topic],
                    "-p",
                    ["prompt:=", prompt],
                    "-p",
                    ["prompt_topic:=", prompt_topic],
                    "-p",
                    ["active_prompt_topic:=", active_prompt_topic],
                    "-p",
                    ["device:=", sam3_device],
                    "-p",
                    ["infer_hz:=", sam3_infer_hz],
                    "-p",
                    ["max_side:=", sam3_max_side],
                    "-p",
                    ["score_th:=", sam3_score_th],
                    "-p",
                    ["mask_th:=", sam3_mask_th],
                ],
            ),
            Node(
                package="perception_geometry",
                executable="mask_depth_fusion_node",
                name="mask_depth_fusion_node",
                output="screen",
                parameters=[
                    {
                        "mask_topic": "/sam3/mask",
                        "score_topic": "/sam3/score",
                        "prompt_topic": active_prompt_topic,
                        "color_topic": color_topic,
                        "depth_topic": depth_topic,
                        "camera_info_topic": camera_info_topic,
                        "overlay_topic": overlay_topic,
                        "target_frame": target_frame,
                        "depth_scale": ParameterValue(depth_scale, value_type=float),
                        "min_score": ParameterValue(min_score, value_type=float),
                        "max_frame_age_sec": ParameterValue(max_frame_age_sec, value_type=float),
                        "cluster_count": ParameterValue(cluster_count, value_type=int),
                        "cluster_max_samples": ParameterValue(cluster_max_samples, value_type=int),
                        "cluster_meanshift_bandwidth_m": ParameterValue(
                            cluster_meanshift_bandwidth_m,
                            value_type=float,
                        ),
                        "use_top_surface_estimator": ParameterValue(
                            use_top_surface_estimator,
                            value_type=bool,
                        ),
                        "top_surface_band_m": ParameterValue(
                            top_surface_band_m,
                            value_type=float,
                        ),
                        "top_surface_min_fraction": ParameterValue(
                            top_surface_min_fraction,
                            value_type=float,
                        ),
                        "top_surface_percentile": ParameterValue(
                            top_surface_percentile,
                            value_type=float,
                        ),
                        "keypoint_filter_alpha": ParameterValue(
                            keypoint_filter_alpha,
                            value_type=float,
                        ),
                        "keypoint_jump_reset_m": ParameterValue(
                            keypoint_jump_reset_m,
                            value_type=float,
                        ),
                        "keypoint_jump_hold_frames": ParameterValue(
                            keypoint_jump_hold_frames,
                            value_type=int,
                        ),
                        "candidate_lock_radius_m": ParameterValue(
                            candidate_lock_radius_m,
                            value_type=float,
                        ),
                        "invalid_reset_frames": ParameterValue(
                            invalid_reset_frames,
                            value_type=int,
                        ),
                        "prompt_settle_sec": ParameterValue(
                            prompt_settle_sec,
                            value_type=float,
                        ),
                        "enable_container_center_keypoint": ParameterValue(
                            enable_container_center_keypoint,
                            value_type=bool,
                        ),
                        "container_center_inner_fraction": ParameterValue(
                            container_center_inner_fraction,
                            value_type=float,
                        ),
                        "container_center_z_percentile": ParameterValue(
                            container_center_z_percentile,
                            value_type=float,
                        ),
                        "container_center_z_offset_m": ParameterValue(
                            container_center_z_offset_m,
                            value_type=float,
                        ),
                        "container_center_min_points": ParameterValue(
                            container_center_min_points,
                            value_type=int,
                        ),
                        "container_center_expected_world": container_center_expected_world,
                        "container_center_expected_max_distance_m": ParameterValue(
                            container_center_expected_max_distance_m,
                            value_type=float,
                        ),
                    }
                ],
            ),
            Node(
                package="vla_rgbd_tools",
                executable="tracking_overlay_viewer_node",
                name="keypoint_overlay_viewer",
                output="screen",
                condition=IfCondition(start_overlay_viewer),
                parameters=[
                    {
                        "color_topic": color_topic,
                        "depth_topic": depth_topic,
                        "camera_info_topic": camera_info_topic,
                        "overlay_topic": overlay_topic,
                        "prompt_topic": prompt_topic,
                        "score_topic": "/sam3/score",
                        "keypoint_topic": "/perception/keypoint_3d",
                        "keypoint_px_topic": "/perception/keypoint_px",
                        "valid_topic": "/perception/valid",
                        "window_name": viewer_window_name,
                        "refresh_hz": ParameterValue(viewer_refresh_hz, value_type=float),
                        "depth_scale": ParameterValue(depth_scale, value_type=float),
                        "cloud_stride": ParameterValue(viewer_cloud_stride, value_type=int),
                        "cloud_display_max_points": ParameterValue(
                            viewer_cloud_display_max_points,
                            value_type=int,
                        ),
                        "keypoint_timeout_sec": ParameterValue(
                            monitor_keypoint_timeout_sec,
                            value_type=float,
                        ),
                    }
                ],
            ),
            Node(
                package="robot_monitor",
                executable="robot_monitor",
                name="robot_monitor",
                output="screen",
                condition=IfCondition(start_robot_monitor),
                parameters=[
                    {
                        "prompt_topic": prompt_topic,
                        "valid_topic": "/perception/valid",
                        "keypoint_topic": "/perception/keypoint_3d",
                        "candidate_topic": "/perception/keypoint_candidates_text",
                        "keypoint_timeout_sec": ParameterValue(
                            monitor_keypoint_timeout_sec,
                            value_type=float,
                        ),
                    }
                ],
            ),
            Node(
                package="pybullet_ros2_sim",
                executable="scene_object_registry_node",
                name="scene_object_registry_node",
                output="screen",
                parameters=[
                    {
                        "prompt_topic": prompt_topic,
                        "target_frame": target_frame,
                    }
                ],
            ),
        ]
    )
