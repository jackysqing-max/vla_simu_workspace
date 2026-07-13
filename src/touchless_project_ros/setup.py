from glob import glob

from setuptools import find_packages, setup

package_name = "touchless_project"

data_files = [
    (
        "share/ament_index/resource_index/packages",
        ["resource/" + package_name],
    ),
    ("share/" + package_name, ["package.xml"]),
    ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
]

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="siqin",
    maintainer_email="jackysqing@gmail.com",
    description="ROS 2 nodes for the touchless gesture control project",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "touchless_gesture_node = touchless_project.gesture_command_node:main",
            "touchless_mri_viewer_node = touchless_project.mri_viewer_node:main",
            "gesture_ee_pose_node = touchless_project.gesture_ee_pose_node:main",
        ],
    },
)
