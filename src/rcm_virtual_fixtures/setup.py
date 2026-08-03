from glob import glob

from setuptools import find_packages, setup


package_name = "rcm_virtual_fixtures"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        (
            "share/" + package_name + "/meshes",
            glob("meshes/*.stl") + glob("meshes/*.STL"),
        ),
        (
            "share/" + package_name + "/meshes/dvrk_lnd_420006",
            glob("meshes/dvrk_lnd_420006/*"),
        ),
        ("share/" + package_name + "/urdf", glob("urdf/*.urdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="siqin",
    maintainer_email="siqin@todo.todo",
    description=(
        "KUKA iiwa PyBullet demos for RCM, line, and plane virtual fixtures "
        "adapted from Franka RCM experiments."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rcm_virtual_fixture_node = rcm_virtual_fixtures.rcm_virtual_fixture_node:main",
            "vlm_port_pose_node = rcm_virtual_fixtures.vlm_port_pose_node:main",
            "semantic_port_grounder_node = rcm_virtual_fixtures.semantic_port_grounder_node:main",
            "surgical_rcm_task_executor_node = rcm_virtual_fixtures.surgical_rcm_task_executor_node:main",
        ],
    },
)
