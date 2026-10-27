from setuptools import find_packages, setup

package_name = 'qr_detection'

setup(
    name=package_name,
    version='0.1.0',
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
    description='Downward-camera QR detection, ground projection and corridor ground markers for Mission 2',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'qr_scanner_node = qr_detection.qr_scanner_node:main',
            'ground_marker_node = qr_detection.ground_marker_node:main',
            'red_zone_node = qr_detection.red_zone_node:main',
            'sim_down_camera_node = qr_detection.sim_down_camera_node:main',
        ],
    },
)
