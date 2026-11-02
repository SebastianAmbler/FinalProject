"""Run the POD S3 reader and the limit-switch/buzzer S3 reader at the same time.

Each node owns its own serial port and its own timer, so both streams are
read concurrently (separate processes).

  ros2 launch pod_serial_reader both_boards.launch.py \
      pod_port:=/dev/serial/by-id/usb-...POD... \
      limit_port:=/dev/serial/by-id/usb-...LIMIT...

Both nodes redraw the terminal when 'display' is true, which would fight
over one screen, so only the POD table is on by default
(show_pod:=false / show_limit:=true to change).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pod_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('limit_port', default_value='/dev/ttyACM1'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('show_pod', default_value='true'),
        DeclareLaunchArgument('show_limit', default_value='false'),
        Node(
            package='pod_serial_reader',
            executable='serial_reader',
            name='pod_serial_reader',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('pod_port'),
                'baud': LaunchConfiguration('baud'),
                'display': LaunchConfiguration('show_pod'),
            }],
        ),
        Node(
            package='pod_serial_reader',
            executable='limit_buzzer',
            name='limit_buzzer_reader',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('limit_port'),
                'baud': LaunchConfiguration('baud'),
                'display': LaunchConfiguration('show_limit'),
            }],
        ),
    ])
