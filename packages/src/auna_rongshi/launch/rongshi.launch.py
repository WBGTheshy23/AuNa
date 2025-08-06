from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        use_sim_time_arg,
        Node(
            package='auna_rongshi',
            executable='robot_marker_publisher',
            name='robot_marker_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='auna_rongshi',
            executable='robot_marker_clustering',
            name='robot_marker_clustering',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='auna_rongshi',
            executable='robot_mot',
            name='robot_mot',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
# This launch file starts the robot marker publisher, clustering, and MOT nodes.