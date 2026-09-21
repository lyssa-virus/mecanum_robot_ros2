from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    package_dir = get_package_share_directory(
        'mecanum_bringup'
    )

    ps4_config = os.path.join(
        package_dir,
        'config',
        'ps4.config.yaml'
    )

    serial_port = LaunchConfiguration('serial_port')

    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/arduino',
        description='Porta serial do Arduino Mega'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0
        }]
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[ps4_config]
    )

    serial_bridge = Node(
        package='mecanum_serial_bridge',
        executable='serial_bridge',
        name='mecanum_serial_bridge',
        output='screen',
        parameters=[{
            'port': serial_port,
            'baudrate': 115200,
            'send_rate': 20.0,
            'cmd_timeout': 0.5
        }]
    )

    return LaunchDescription([
        declare_serial_port,
        joy_node,
        teleop_node,
        serial_bridge
    ])
