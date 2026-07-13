from setuptools import find_packages, setup

package_name = 'perception_geometry'

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
    description='Mask + depth fusion for 3D keypoint extraction',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'mask_depth_fusion_node = perception_geometry.mask_depth_fusion_node:main',
            'give_me_scissors_keypoint_node = '
            'perception_geometry.give_me_scissors_keypoint_node:main',
        ],
    },
)
