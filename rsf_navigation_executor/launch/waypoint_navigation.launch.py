from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('rsf_navigation_executor'), 'config', 'waypoints.yaml']
        ),
    )
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    waypoint_navigator_node = Node(
        package='rsf_navigation_executor',
        executable='waypoint_navigator.py',
        name='waypoint_navigator',
        parameters=[{
            'waypoints_file': LaunchConfiguration('waypoints_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen',
    )

    return LaunchDescription([
        waypoints_file_arg,
        use_sim_time_arg,
        waypoint_navigator_node,
    ])
