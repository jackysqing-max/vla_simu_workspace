import os
from glob import glob
from pathlib import Path

from setuptools import find_packages, setup

package_name = "touchless_project"
package_root = Path(__file__).resolve().parent
legacy_root = package_root.parent / "touchless_project"
legacy_volume = legacy_root / "t1_150116AR_20150115.nii"

data_files = [
    (
        "share/ament_index/resource_index/packages",
        ["resource/" + package_name],
    ),
    ("share/" + package_name, ["package.xml"]),
    ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
]

if legacy_volume.exists():
    data_files.append(
        (
            "share/" + package_name + "/data",
            [os.path.relpath(legacy_volume, package_root)],
        ),
    )

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
