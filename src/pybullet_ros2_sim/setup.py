from glob import glob

from setuptools import find_packages, setup

package_name = 'pybullet_ros2_sim'

setup(
    name=package_name,
    version='0.1.5',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='siqin',
    maintainer_email='jackysqing@gmail.com',
    description='ROS 2 + PyBullet iiwa simulation, control, and perception nodes',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'iiwa_pybullet_sim_node = pybullet_ros2_sim.iiwa_pybullet_sim_node:main',
            'iiwa_pybullet_rgbd_sim_node = pybullet_ros2_sim.iiwa_pybullet_rgbd_sim_node:main',
            'iiwa_impedance_controller = pybullet_ros2_sim.iiwa_impedance_controller:main',
            'iiwa_line_ik_desired = pybullet_ros2_sim.iiwa_line_ik_desired:main',
            'iiwa_keypoint_tracker_node = pybullet_ros2_sim.iiwa_keypoint_tracker_node:main',
            'scene_object_registry_node = pybullet_ros2_sim.scene_object_registry_node:main',
            'llm_task_planner_node = pybullet_ros2_sim.llm_task_planner_node:main',
            'llm_task_executor_node = pybullet_ros2_sim.llm_task_executor_node:main',
        ],
    },
)
