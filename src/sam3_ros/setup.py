from setuptools import find_packages, setup

package_name = 'sam3_ros'

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
    maintainer_email='jackysqing@gmail.com',
    description='Prompt-driven SAM3 segmentation for ROS 2 image topics',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sam3_mask_node = sam3_ros.sam3_mask_node:main',
        ],
    },
)
