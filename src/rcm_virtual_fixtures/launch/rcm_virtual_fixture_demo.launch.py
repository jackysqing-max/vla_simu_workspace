import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fixture_mode = LaunchConfiguration("fixture_mode")
    gui = LaunchConfiguration("gui")
    publish_hz = LaunchConfiguration("publish_hz")
    speed_mps = LaunchConfiguration("speed_mps")
    trajectory_radius_m = LaunchConfiguration("trajectory_radius_m")
    trajectory_length_m = LaunchConfiguration("trajectory_length_m")
    tool_length_m = LaunchConfiguration("tool_length_m")
    show_tool = LaunchConfiguration("show_tool")
    tool_radius_m = LaunchConfiguration("tool_radius_m")
    show_phantom = LaunchConfiguration("show_phantom")
    phantom_collision = LaunchConfiguration("phantom_collision")
    rcm_lambda = LaunchConfiguration("rcm_lambda")
    max_joint_step_rad = LaunchConfiguration("max_joint_step_rad")
    phantom_mesh = os.path.join(
        get_package_share_directory("rcm_virtual_fixtures"),
        "meshes",
        "phantom_centered.stl",
    )
    dvrk_lnd_urdf = os.path.join(
        get_package_share_directory("rcm_virtual_fixtures"),
        "urdf",
        "dvrk_lnd_420006_tip.urdf",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fixture_mode",
                default_value="rcm",
                description="Virtual fixture mode: rcm, line, or plane.",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Open the PyBullet GUI.",
            ),
            DeclareLaunchArgument(
                "publish_hz",
                default_value="100.0",
                description="Joint target publish rate.",
            ),
            DeclareLaunchArgument(
                "speed_mps",
                default_value="0.018",
                description="Task-space trajectory speed.",
            ),
            DeclareLaunchArgument(
                "trajectory_radius_m",
                default_value="0.020",
                description="Circle radius for RCM and plane modes.",
            ),
            DeclareLaunchArgument(
                "trajectory_length_m",
                default_value="0.100",
                description="Line travel distance for line mode.",
            ),
            DeclareLaunchArgument(
                "tool_length_m",
                default_value="0.220",
                description="Virtual tool length from KUKA wrist to tool tip.",
            ),
            DeclareLaunchArgument(
                "show_tool",
                default_value="true",
                description="Show the rigid surgical tool attached to link 7.",
            ),
            DeclareLaunchArgument(
                "tool_radius_m",
                default_value="0.006",
                description="Radius of the visual surgical tool shaft.",
            ),
            DeclareLaunchArgument(
                "show_phantom",
                default_value="true",
                description="Load the RCM phantom STL in the PyBullet scene.",
            ),
            DeclareLaunchArgument(
                "phantom_collision",
                default_value="false",
                description="Enable static concave collision for the phantom.",
            ),
            DeclareLaunchArgument(
                "rcm_lambda",
                default_value="0.650",
                description="RCM interpolation factor between wrist and virtual tip.",
            ),
            DeclareLaunchArgument(
                "max_joint_step_rad",
                default_value="0.018",
                description="Per-cycle joint target step clamp for IK continuity.",
            ),
            Node(
                package="pybullet_ros2_sim",
                executable="iiwa_pybullet_sim_node",
                name="iiwa_pybullet_sim_node",
                output="screen",
                parameters=[
                    {
                        "gui": gui,
                        "use_goto": True,
                        "init_q": [
                            0.0,
                            0.462931412,
                            0.0,
                            -1.132967496,
                            0.0,
                            1.204016934,
                            0.0,
                        ],
                        "reset_to_init_on_start": True,
                        "goto_duration": 0.5,
                        "goto_use_reset": True,
                        "goto_force": 260.0,
                        "goto_pos_gain": 0.65,
                        "goto_vel_gain": 1.0,
                        "goto_max_vel": 6.0,
                        "track_force": 260.0,
                        "track_pos_gain": 0.65,
                        "track_vel_gain": 1.00,
                        "track_max_vel": 6.0,
                        "table_x_m": 0.9,
                        "table_y_m": 0.0,
                        "table_top_z_m": 0.240701,
                        "clean_gui": True,
                        "camera_distance_m": 0.90,
                        "camera_yaw_deg": 42.0,
                        "camera_pitch_deg": -38.0,
                        "show_rcm_debug_markers": True,
                        "rcm_marker_radius_m": 0.004,
                        "rcm_tip_marker_radius_m": 0.002,
                        "ee_trace_point_radius_m": 0.0025,
                        "rcm_trace_min_inserted_depth_m": 0.065,
                        "show_port_detection_overlay": True,
                        "port_overlay_ring_radius_m": 0.010,
                        "port_overlay_axis_outside_m": 0.050,
                        "port_overlay_axis_inside_m": 0.090,
                        "show_rcm_tool": show_tool,
                        "rcm_tool_length_m": tool_length_m,
                        "rcm_tool_radius_m": tool_radius_m,
                        "show_dvrk_lnd_gripper": True,
                        "dvrk_lnd_urdf_path": dvrk_lnd_urdf,
                        "dvrk_lnd_jaw_angle_rad": 0.45,
                        "dvrk_lnd_tip_offset_m": 0.026,
                        "trace_rcm_tool_tip": True,
                        "show_rcm_phantom": show_phantom,
                        "rcm_phantom_mesh_path": phantom_mesh,
                        "rcm_phantom_collision": phantom_collision,
                        "rcm_phantom_alpha": 0.50,
                        "rcm_phantom_x_m": 0.701726,
                        "rcm_phantom_y_m": 0.0,
                        "rcm_phantom_z_m": 0.240701,
                    }
                ],
            ),
            Node(
                package="rcm_virtual_fixtures",
                executable="rcm_virtual_fixture_node",
                name="rcm_virtual_fixture_node",
                output="screen",
                parameters=[
                    {
                        "fixture_mode": fixture_mode,
                        "publish_hz": publish_hz,
                        "speed_mps": speed_mps,
                        "trajectory_radius_m": trajectory_radius_m,
                        "trajectory_length_m": trajectory_length_m,
                        "tool_length_m": tool_length_m,
                        "rcm_lambda": rcm_lambda,
                        "use_initial_rcm": False,
                        "rcm_world": [0.701726, 0.0, 0.404701],
                        "enable_safe_insertion": True,
                        "preinsert_clearance_m": 0.040,
                        "port_standoff_m": 0.012,
                        "preinsert_hold_sec": 1.0,
                        "approach_duration_sec": 2.0,
                        "port_dwell_sec": 0.8,
                        "insertion_speed_mps": 0.018,
                        "inserted_dwell_sec": 0.8,
                        "stage_position_tolerance_m": 0.006,
                        "max_joint_step_rad": max_joint_step_rad,
                    }
                ],
            ),
        ]
    )
