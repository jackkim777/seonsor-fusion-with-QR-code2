import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

cartographer_package = 'turtlebot3_cartographer'

def generate_launch_description():
    node1 = Node(
        package='qr_localization',
        executable='qr_camera',
        name='qr_camera',
        output='screen'
    )
    
    node2 = Node(
        package='qr_localization',
        executable='z_coord',
        name='gaussian_mean_calculator',
        output='screen'
    )

    node3 = Node(
        package='qr_localization',
        executable='QrToRobot',
        name='QrToRobot',
        output='screen'
    )

    node4 = Node(
        package='qr_localization',
        executable='QR_Position',
        name='QR_Position',
        output='screen'
    )



    return LaunchDescription([
        node1,
        node2,
        node3,
        node4,
    ])

if __name__ == '__main__':
    generate_launch_description()
