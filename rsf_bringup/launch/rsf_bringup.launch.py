from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_param_path = PathJoinSubstitution([
        FindPackageShare('rsf_bringup'),
        'config',
        'bringup_params.yaml',
    ])

    use_joy = LaunchConfiguration('use_joy')
    use_icart = LaunchConfiguration('use_icart')
    use_hokuyo = LaunchConfiguration('use_hokuyo')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_joy',
            default_value='true',
            description='Start joy_node and teleop_twist_joy.',
        ),
        DeclareLaunchArgument(
            'use_icart',
            default_value='false',
            description='Include the icart_driver launch file.',
        ),
        DeclareLaunchArgument(
            'use_hokuyo',
            default_value='false',
            description='Start the hokuyo_rsf driver node.',
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[bringup_param_path],
            output='screen',
            condition=IfCondition(use_joy),
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[bringup_param_path],
            output='screen',
            condition=IfCondition(use_joy),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('icart_driver'),
                    'launch',
                    'icart_drive.launch.py',
                ])
            ),
            condition=IfCondition(use_icart),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('rsf_description'),
                    'launch',
                    'display.launch.py',
                ])
            ),
        ),
        Node(
            package='rsf_bringup',
            executable='tf_odom_to_footprint',
            name='tf_odom_to_footprint',
            output='screen',
        ),
        Node(
            package='hokuyo_rsf',
            executable='hokuyo_rsf',
            name='hokuyo_rsf',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('hokuyo_rsf'),
                    'config',
                    'hokuyo_rsf.yaml',
                ]),
                {'param_files_dir': PathJoinSubstitution([
                    FindPackageShare('hokuyo_rsf'),
                    'config',
                ])},
                {'broadcast_tf': False},
            ],
            condition=IfCondition(use_hokuyo),
        ),
    ])
