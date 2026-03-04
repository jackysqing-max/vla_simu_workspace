from setuptools import find_packages, setup

package_name = 'iiwa_state_udp_bridge'

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
    description='Bridge iiwa joint state/torque/desired to RobotState + UDP for monitor',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'robotstate_bridge = iiwa_state_udp_bridge.robotstate_bridge:main',
            'bridge_node = iiwa_state_udp_bridge.bridge_node:main',
        ],
    },
)
