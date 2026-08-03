import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    simulator_dir = get_package_share_directory('rsf_simulator')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='tsudanuma2-3',
        choices=['tsudanuma2-3', 'tsudanuma'],
        description='World in rsf_simulator/worlds: tsudanuma2-3 (building editor) '
                    'or tsudanuma (generated from an occupancy grid map by map2sdf)'
    )

    interlace_arg = DeclareLaunchArgument(
        'interlace',
        default_value='1',
        description='Horizontal interlace factor (1-20)'
    )

    world_file = PathJoinSubstitution([
        simulator_dir, 'worlds', [LaunchConfiguration('world'), '.sdf']
    ])

    set_resource_path = AppendEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        os.path.dirname(simulator_dir)
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments=[('gz_args', ['-r -v 4 ', world_file])]
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/rsf/hokuyo3d/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/rsf/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/rsf/nav_sat_fix@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
        ],
        remappings=[
            ('/odom', '/rsf/rsf_odom'),
        ],
        output='screen',
    )

    deskew_node = Node(
        package='rsf_simulator',
        executable='interlace_deskew_node',
        parameters=[{
            'interlace': ParameterValue(LaunchConfiguration('interlace'), value_type=int),
            'use_sim_time': True,
        }],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        interlace_arg,
        set_resource_path,
        gazebo,
        bridge_node,
        deskew_node,
    ])
