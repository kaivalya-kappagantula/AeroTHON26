from setuptools import find_packages, setup

package_name = 'payload'

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
    description='Payload release servo (/payload/release) via MAV_CMD_DO_SET_SERVO.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'payload_node = payload.payload_node:main',
        ],
    },
)
