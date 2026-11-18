import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('xbee_teleop'), 'config', 'params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('arduino', default_value='false',
                              description='Launch Arduino Mega serial bridge node as well'),

        # Network UDP Receiver Node (Port 8889)
        Node(
            package='xbee_teleop',
            executable='science_rx_node',
            name='science_rx_node',
            output='screen',
            parameters=[cfg]
        ),

        # Optional Arduino Mega Serial Bridge (subscribes to /science/actuator_cmd)
        Node(
            package='xbee_teleop',
            executable='science_node',
            name='science_node',
            output='screen',
            parameters=[cfg],
            condition=IfCondition(LaunchConfiguration('arduino'))
        ),
    ])
