from setuptools import find_packages, setup

package_name = 'mecanum_serial_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=[
        'setuptools',
        'pyserial',
    ],
    zip_safe=True,
    maintainer='tigo',
    maintainer_email='tigo@todo.todo',
    description='Bridge ROS2 cmd_vel para Arduino Mega via serial',
    license='MIT',
    entry_points={
        'console_scripts': [
            'serial_bridge = mecanum_serial_bridge.serial_bridge:main',
            'custom_teleop = mecanum_serial_bridge.custom_teleop:main',
        ],
    },
)
