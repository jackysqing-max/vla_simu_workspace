from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params_file = PathJoinSubstitution(
        [
            FindPackageShare("pybullet_ros2_sim"),
            "config",
            "llm_task_planner_qwen3_vllm.yaml",
        ]
    )

    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="ROS2 parameter file for the local Qwen3 planner backend.",
            ),
            Node(
                package="pybullet_ros2_sim",
                executable="llm_task_planner_node",
                name="llm_task_planner_node",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
