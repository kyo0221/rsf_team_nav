import os
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    package_share_dir = get_package_share_directory('rsf_bringup')
    bringup_param_path = os.path.join(package_share_dir, 'config', 'bringup_params.yaml')

    with open(bringup_param_path, 'r') as file:
        launch_params = yaml.safe_load(file)['launch']['ros__parameters']

    # hokuyo_rsfの起動コマンドの作成
    hokuyo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hokuyo_rsf'),
                'launch',
                'hokuyo_rsf.launch.py'
            )
        )
    )

    # コントローラーの起動コマンドの作成
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[bringup_param_path],
        output='screen'
    )
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[bringup_param_path],
        output='screen'
    )

    # 起動エンティティクラスの作成
    launch_description = LaunchDescription()

    # 起動の追加
    if launch_params.get('joy', True):
        launch_description.add_entity(joy_node)
        launch_description.add_entity(teleop_node)
    if launch_params.get('icart', True):
        icart_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('icart_driver'),
                    'launch',
                    'icart_drive.launch.py'
                )
            )
        )
        launch_description.add_entity(icart_launch)

    launch_description.add_entity(hokuyo_launch)

    return launch_description
