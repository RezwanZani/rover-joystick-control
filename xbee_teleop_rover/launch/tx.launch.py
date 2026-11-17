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
        DeclareLaunchArgument('gui', default_value='true'),
        Node(package='joy', executable='joy_node', parameters=[cfg]),
        Node(package='xbee_teleop', executable='tx_node', output='screen', parameters=[cfg]),
        Node(package='xbee_teleop', executable='gui', condition=IfCondition(LaunchConfiguration('gui'))),
    ])
