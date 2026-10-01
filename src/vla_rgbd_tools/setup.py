from glob import glob

from setuptools import find_packages, setup

package_name = 'vla_rgbd_tools'

setup(
    name=package_name,
    version='0.1.5',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='siqin',
    maintainer_email='jackysqing@gmail.com',
    description='Real RGB-D launch and visualization tools for the VLA keypoint pipeline',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mask_depth_to_cloud_node = vla_rgbd_tools.mask_depth_to_cloud_node:main',
            'tracking_overlay_viewer_node = vla_rgbd_tools.tracking_overlay_viewer_node:main',
        ],
    },
)
