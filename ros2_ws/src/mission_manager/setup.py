from setuptools import find_packages, setup

package_name = 'mission_manager'

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
    maintainer='jaya9899',
    maintainer_email='jaya9899@todo.todo',
    description='Mission 2 state machine plus SITL QR simulator and flight-primitive test nodes.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mission_node = mission_manager.mission_node:main',
            'sitl_qr_simulator = mission_manager.sitl_qr_simulator:main',
            'flight_primitives = mission_manager.flight_primitives:main',
            'qr_scan_mission_node = mission_manager.qr_scan_mission_node:main',
            'takeoff_test = mission_manager.takeoff_test:main',
            'entry_corridor_mission = mission_manager.entry_corridor_mission:main',
            'arena_search_mission = mission_manager.arena_search_mission:main',
            'bcd_sweep_mission = mission_manager.bcd_sweep_mission:main',
            'bcd_redzone_mission = mission_manager.bcd_redzone_mission:main',
            'qr_bcd_mission = mission_manager.qr_bcd_mission:main',
        ],
    },
)
