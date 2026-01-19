 #!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2024, UFACTORY, Inc.
# All rights reserved.
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ros_namespace = LaunchConfiguration('ros_namespace', default='')

    return LaunchDescription([
        DeclareLaunchArgument(
            'ros_namespace',
            default_value='',
            description='Optional ROS namespace for the node.'
        ),
        Node(
            package='xarm_moveit_servo',
            executable='xarm_track_pose',
            name='xarm_track_pose',
            namespace=ros_namespace,
            parameters=[
            ],
        ),
    ])
