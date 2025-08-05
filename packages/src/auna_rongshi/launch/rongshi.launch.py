from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='auna_rongshi',
            executable='robot_marker_publisher',
            name='robot_marker_publisher',
            output='screen'
        ),
        Node(
            package='auna_rongshi',
            executable='robot_marker_clustering',
            name='robot_marker_clustering',
            output='screen'
        ),
        Node(
            package='auna_rongshi',
            executable='robot_mot',
            name='robot_mot',
            output='screen'
        ),
    ])
# This launch file starts the robot marker publisher, clustering, and MOT nodes.