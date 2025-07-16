#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # パッケージディレクトリの取得
    package_dir = get_package_share_directory('odrive_can_driver')
    
    # 設定ファイルのパス
    config_file = os.path.join(package_dir, 'config', 'odrive_can_driver.yaml')
    
    # ラウンチ引数の宣言
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN interface name'
    )
    
    left_node_id_arg = DeclareLaunchArgument(
        'left_wheel_node_id',
        default_value='1',
        description='Left wheel ODrive node ID'
    )
    
    right_node_id_arg = DeclareLaunchArgument(
        'right_wheel_node_id',
        default_value='2',
        description='Right wheel ODrive node ID'
    )
    
    # ODrive CAN Driverノード
    odrive_can_driver_node = Node(
        package='odrive_can_driver',
        executable='odrive_can_driver_node',
        name='odrive_can_driver',
        parameters=[
            config_file,
            {
                'can_interface': LaunchConfiguration('can_interface'),
                'left_wheel_node_id': LaunchConfiguration('left_wheel_node_id'),
                'right_wheel_node_id': LaunchConfiguration('right_wheel_node_id'),
            }
        ],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        can_interface_arg,
        left_node_id_arg,
        right_node_id_arg,
        odrive_can_driver_node,
    ])
