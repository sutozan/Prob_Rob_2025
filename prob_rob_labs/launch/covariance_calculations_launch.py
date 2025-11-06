import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),
        DeclareLaunchArgument('landmark_name', default_value='landmark_5::link',
                              description='set landmark being observed'),
        DeclareLaunchArgument('color', default_value='cyan',
                              description='set color for topic name'),
        DeclareLaunchArgument('dx_camera', default_value='0.076',
                              description='set camera offset in x-axis '),
        DeclareLaunchArgument('dy_camera', default_value='0.0',
                              description='set camera offset in y-axis'),
        Node(
            package='prob_rob_labs',
            executable='covariance_calculations',
            name='covariance_calculations',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'landmark_name': LaunchConfiguration('landmark_name')},
                        {'color': LaunchConfiguration('color')},
                        {'dx_camera': LaunchConfiguration('dx_camera')},
                        {'dy_camera': LaunchConfiguration('dy_camera')},]
        )
    ])
