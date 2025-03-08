from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 定义激光避障节点
    laser_Avoidance_node = Node(
        package='yahboomcar_laser',  # 包名称
        executable='laser_Avoidance_a1_X3',  # 可执行文件名称
    )


    # 包含LiDAR节点的启动文件
    lidar_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_c1_launch.py')
        ])
    )

    # 包含小车启动节点的启动文件
    pwm_node = Node(
       package='control',
       executable='pwm_node',
    )

    # 创建并返回Launch描述对象，包含上述三个节点
    launch_description = LaunchDescription([laser_Avoidance_node, lidar_node, pwm_node])
    return launch_description

