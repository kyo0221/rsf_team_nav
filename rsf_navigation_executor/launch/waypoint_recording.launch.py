from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    output_file_arg = DeclareLaunchArgument(
        'output_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('rsf_navigation_executor'), 'config', 'recorded_waypoints.yaml']
        ),
    )
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    waypoint_recorder_node = Node(
        package='rsf_navigation_executor',
        executable='waypoint_recorder.py',
        name='waypoint_recorder',
        parameters=[{
            'output_file': LaunchConfiguration('output_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen',
    )

    return LaunchDescription([
        output_file_arg,
        use_sim_time_arg,
        waypoint_recorder_node,
    ])
