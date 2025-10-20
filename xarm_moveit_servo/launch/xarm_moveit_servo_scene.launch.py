 #!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2024, UFACTORY, Inc.
# All rights reserved.
#
# Author: OpenAI Assistant

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    planning_frame = LaunchConfiguration('planning_frame', default='link_base')
    apply_delay = LaunchConfiguration('apply_delay', default='1.0')
    ros_namespace = LaunchConfiguration('ros_namespace', default='')

    return LaunchDescription([
        DeclareLaunchArgument(
            'planning_frame',
            default_value='link_base',
            description='Planning frame for the collision objects (defaults to link_base).'
        ),
        DeclareLaunchArgument(
            'apply_delay',
            default_value='1.0',
            description='Seconds to wait after applying the collision objects before shutting down.'
        ),
        DeclareLaunchArgument(
            'ros_namespace',
            default_value='',
            description='Optional ROS namespace for the node.'
        ),
        Node(
            package='xarm_moveit_servo',
            executable='xarm_servo_scene',
            name='xarm_servo_scene',
            namespace=ros_namespace,
            output='screen',
            parameters=[
                {
                    'planning_frame': planning_frame,
                    'apply_delay': apply_delay,
                }
            ],
        ),
    ])
