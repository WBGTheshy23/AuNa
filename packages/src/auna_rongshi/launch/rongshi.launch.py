from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

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
    ])
# This launch file starts the robot marker publisher, clustering, and MOT nodes.