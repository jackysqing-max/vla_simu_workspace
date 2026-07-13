from setuptools import find_packages, setup

package_name = 'udp_realtime_plot'

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
            'udp_receiver = udp_realtime_plot.udp_receiver_node:main',
            'udp_plotter = udp_realtime_plot.realtime_plot_node:main',
        ],
    },
)
