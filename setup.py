from glob import glob
from setuptools import setup

package_name = 'ros2_nav_sim'

setup(
    name=package_name,
    version='2.1.0',
    packages=[
        package_name,
        f'{package_name}.msgs',
        f'{package_name}.actions',
        f'{package_name}.nodes',
        f'{package_name}.bringup',
        f'{package_name}.bringup.launch',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('ros2_nav_sim/bringup/launch/*.launch.py')),
        ('share/' + package_name + '/config', ['ros2_nav_sim/config/nav2_params.yaml']),
    ],
    install_requires=['setuptools', 'pygame', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='ros2_nav_sim',
    maintainer_email='dev@ros2-nav-sim.dev',
    description='Cyberpunk-style ROS 2 Navigation Simulator',
    license='MIT',
    entry_points={
        'console_scripts': [
            'nav_simulator = ros2_nav_sim.main:main',
        ],
    },
)
