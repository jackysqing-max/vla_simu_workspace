from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_index = LaunchConfiguration("camera_index")
    nifti_path = LaunchConfiguration("nifti_path")
    show_debug_window = LaunchConfiguration("show_debug_window")
    start_viewer = LaunchConfiguration("start_viewer")
    sensitivity = LaunchConfiguration("sensitivity")

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_index", default_value="0"),
            DeclareLaunchArgument("nifti_path", default_value=""),
            DeclareLaunchArgument("show_debug_window", default_value="true"),
            DeclareLaunchArgument("start_viewer", default_value="true"),
            DeclareLaunchArgument("sensitivity", default_value="1.0"),
            Node(
                package="touchless_project",
                executable="touchless_gesture_node",
                name="touchless_gesture_node",
                output="screen",
                parameters=[
                    {
                        "camera_index": camera_index,
                        "show_debug_window": show_debug_window,
                        "sensitivity": sensitivity,
                    }
                ],
            ),
            Node(
                package="touchless_project",
                executable="touchless_mri_viewer_node",
                name="touchless_mri_viewer_node",
                output="screen",
                condition=IfCondition(start_viewer),
                parameters=[
                    {
                        "nifti_path": nifti_path,
                    }
                ],
            ),
        ]
    )
