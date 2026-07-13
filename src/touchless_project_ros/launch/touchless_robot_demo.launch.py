from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    camera_index = LaunchConfiguration("camera_index")
    show_debug_window = LaunchConfiguration("show_debug_window")
    sensitivity = LaunchConfiguration("sensitivity")
    start_sim = LaunchConfiguration("start_sim")
    sim_gui = LaunchConfiguration("sim_gui")
    start_state_bridge = LaunchConfiguration("start_state_bridge")
    start_robot_monitor = LaunchConfiguration("start_robot_monitor")
    start_cpp_impedance_controller = LaunchConfiguration("start_cpp_impedance_controller")
    use_cpp_kinematics = LaunchConfiguration("use_cpp_kinematics")
    control_mode_value = LaunchConfiguration("control_mode_value")
    pan_y_m_per_px = LaunchConfiguration("pan_y_m_per_px")
    pan_z_m_per_px = LaunchConfiguration("pan_z_m_per_px")
    zoom_x_m_per_px = LaunchConfiguration("zoom_x_m_per_px")
    rotate_pitch_rad_per_px = LaunchConfiguration("rotate_pitch_rad_per_px")
    rotate_yaw_rad_per_px = LaunchConfiguration("rotate_yaw_rad_per_px")

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_index", default_value="0"),
            DeclareLaunchArgument("show_debug_window", default_value="true"),
            DeclareLaunchArgument("sensitivity", default_value="1.0"),
            DeclareLaunchArgument("start_sim", default_value="true"),
            DeclareLaunchArgument("sim_gui", default_value="true"),
            DeclareLaunchArgument("start_state_bridge", default_value="true"),
            DeclareLaunchArgument("start_robot_monitor", default_value="true"),
            DeclareLaunchArgument("start_cpp_impedance_controller", default_value="false"),
            DeclareLaunchArgument("use_cpp_kinematics", default_value="true"),
            DeclareLaunchArgument("control_mode_value", default_value="1"),
            DeclareLaunchArgument("pan_y_m_per_px", default_value="0.0015"),
            DeclareLaunchArgument("pan_z_m_per_px", default_value="0.0015"),
            DeclareLaunchArgument("zoom_x_m_per_px", default_value="0.0015"),
            DeclareLaunchArgument("rotate_pitch_rad_per_px", default_value="0.004"),
            DeclareLaunchArgument("rotate_yaw_rad_per_px", default_value="0.004"),
            Node(
                package="pybullet_ros2_sim",
                executable="iiwa_pybullet_sim_node",
                name="iiwa_pybullet_sim_node",
                output="screen",
                condition=IfCondition(start_sim),
                parameters=[
                    {
                        "gui": ParameterValue(sim_gui, value_type=bool),
                    }
                ],
            ),
            Node(
                package="touchless_project",
                executable="touchless_gesture_node",
                name="touchless_gesture_node",
                output="screen",
                parameters=[
                    {
                        "camera_index": ParameterValue(camera_index, value_type=int),
                        "show_debug_window": ParameterValue(
                            show_debug_window,
                            value_type=bool,
                        ),
                        "sensitivity": ParameterValue(sensitivity, value_type=float),
                    }
                ],
            ),
            Node(
                package="iiwa_cpp_controller",
                executable="gesture_ee_pose_node_cpp",
                name="gesture_ee_pose_node",
                output="screen",
                condition=IfCondition(use_cpp_kinematics),
                parameters=[
                    {
                        "pan_y_m_per_px": ParameterValue(
                            pan_y_m_per_px,
                            value_type=float,
                        ),
                        "pan_z_m_per_px": ParameterValue(
                            pan_z_m_per_px,
                            value_type=float,
                        ),
                        "zoom_x_m_per_px": ParameterValue(
                            zoom_x_m_per_px,
                            value_type=float,
                        ),
                        "rotate_pitch_rad_per_px": ParameterValue(
                            rotate_pitch_rad_per_px,
                            value_type=float,
                        ),
                        "rotate_yaw_rad_per_px": ParameterValue(
                            rotate_yaw_rad_per_px,
                            value_type=float,
                        ),
                        "control_mode_value": ParameterValue(
                            control_mode_value,
                            value_type=int,
                        ),
                    }
                ],
            ),
            Node(
                package="touchless_project",
                executable="gesture_ee_pose_node",
                name="gesture_ee_pose_node",
                output="screen",
                condition=UnlessCondition(use_cpp_kinematics),
                parameters=[
                    {
                        "pan_y_m_per_px": ParameterValue(
                            pan_y_m_per_px,
                            value_type=float,
                        ),
                        "pan_z_m_per_px": ParameterValue(
                            pan_z_m_per_px,
                            value_type=float,
                        ),
                        "zoom_x_m_per_px": ParameterValue(
                            zoom_x_m_per_px,
                            value_type=float,
                        ),
                        "rotate_pitch_rad_per_px": ParameterValue(
                            rotate_pitch_rad_per_px,
                            value_type=float,
                        ),
                        "rotate_yaw_rad_per_px": ParameterValue(
                            rotate_yaw_rad_per_px,
                            value_type=float,
                        ),
                        "control_mode_value": ParameterValue(
                            control_mode_value,
                            value_type=int,
                        ),
                    }
                ],
            ),
            Node(
                package="iiwa_cpp_controller",
                executable="iiwa_impedance_controller_cpp",
                name="iiwa_impedance_controller_cpp",
                output="screen",
                condition=IfCondition(start_cpp_impedance_controller),
            ),
            Node(
                package="iiwa_state_udp_bridge",
                executable="robotstate_bridge",
                name="robotstate_bridge",
                output="screen",
                condition=IfCondition(start_state_bridge),
            ),
            Node(
                package="robot_monitor",
                executable="robot_monitor",
                name="robot_monitor",
                output="screen",
                condition=IfCondition(start_robot_monitor),
            ),
        ]
    )
