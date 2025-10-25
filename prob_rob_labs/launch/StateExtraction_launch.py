import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='set to true for simulation'),
        DeclareLaunchArgument('frame_of_reference', default_value='odom', description='Frame of reference for ground truth topics'),
        
        Node(
            package='prob_rob_labs',
            executable='StateExtraction',
            name='StateExtraction',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'frame_of_reference': LaunchConfiguration('frame_of_reference')}]
        )
    ])
