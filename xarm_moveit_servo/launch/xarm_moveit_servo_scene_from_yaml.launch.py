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
    planning_frame = LaunchConfiguration('planning_frame', default='link_base')
    ros_namespace = LaunchConfiguration('ros_namespace', default='')
    scene_spec_file = LaunchConfiguration('scene_spec_file', default='')

    return LaunchDescription([
        DeclareLaunchArgument(
            'planning_frame',
            default_value='link_base',
            description='Planning frame for the collision objects (defaults to link_base).'
        ),
        DeclareLaunchArgument(
            'ros_namespace',
            default_value='',
            description='Optional ROS namespace for the node.'
        ),
        DeclareLaunchArgument(
            'scene_spec_file',
            default_value='',
            description='Absolute path to a YAML file defining the planning scene objects.'
        ),
        Node(
            package='xarm_moveit_servo',
            executable='xarm_servo_scene_from_yaml',
            name='xarm_servo_scene_from_yaml',
            namespace=ros_namespace,
            output='screen',
            parameters=[
                {
                    'planning_frame': planning_frame,
                    'scene_spec_file': scene_spec_file,
                }
            ],
        ),
    ])
