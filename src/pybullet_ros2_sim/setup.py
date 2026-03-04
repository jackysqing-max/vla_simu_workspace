from setuptools import find_packages, setup

package_name = 'pybullet_ros2_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='siqin',
    maintainer_email='siqin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'robot_sim_node = pybullet_ros2_sim.robot_sim_node:main',
        'iiwa_cmd_publisher = pybullet_ros2_sim.iiwa_cmd_publisher:main',
        'iiwa_pybullet_sim_node = pybullet_ros2_sim.iiwa_pybullet_sim_node:main',
        'iiwa_circle_ik_desired = pybullet_ros2_sim.iiwa_circle_ik_desired:main',
        'iiwa_impedance_controller = pybullet_ros2_sim.iiwa_impedance_controller:main',
        'iiwa_line_ik_desired = pybullet_ros2_sim.iiwa_line_ik_desired:main',
        ],
    },
)
