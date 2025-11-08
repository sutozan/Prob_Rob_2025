import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),
        DeclareLaunchArgument('color', default_value='cyan',
                              description='set color for topic name'),
        Node(
            package='prob_rob_labs',
            executable='variance_estimation',
            name='variance_estimation',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'color': LaunchConfiguration('color')}]
        )
    ])
