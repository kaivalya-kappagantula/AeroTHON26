import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'obstacle_avoidance'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'params'),
            glob('params/*.param') + glob('params/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaya9899',
    maintainer_email='jaya9899@todo.todo',
    description='Forward-facing OAK-D depth: ArduPilot proximity (sector builder), sensor health (coverage guard) and corridor perception.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'synthetic_sector_node = obstacle_avoidance.synthetic_sector_node:main',
            'obstacle_publisher_node = obstacle_avoidance.obstacle_publisher_node:main',
            'sector_builder_node = obstacle_avoidance.sector_builder_node:main',
            'coverage_guard_node = obstacle_avoidance.coverage_guard_node:main',
            'sector_viz = obstacle_avoidance.sector_viz:main',
            'corridor_perception_node = obstacle_avoidance.corridor_perception_node:main',
            'banner_node = obstacle_avoidance.banner_node:main',
            'sitl_corridor_sim = obstacle_avoidance.sitl_corridor_sim:main',
        ],
    },
)
