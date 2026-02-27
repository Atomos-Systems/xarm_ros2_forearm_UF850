#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _load_eef_params_from_yaml(config_path):
    """Load EEF params from YAML file. Returns dict with string values for launch_arguments."""
    if not os.path.isfile(config_path):
        return {}
    with open(config_path, 'r') as f:
        data = yaml.safe_load(f)
    if not data:
        return {}
    result = {}
    for k, v in data.items():
        if v is True:
            result[k] = 'true'
        elif v is False:
            result[k] = 'false'
        else:
            result[k] = str(v)
    return result


def _launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration('eef_config_file', default='').perform(context)
    if not config_file:
        config_file = os.path.join(
            get_package_share_directory('xarm_moveit_servo'),
            'config', 'uf850_eef_params.yaml'
        )
    eef_params = _load_eef_params_from_yaml(config_file)

    robot_ip = LaunchConfiguration('robot_ip')
    report_type = LaunchConfiguration('report_type', default='dev')
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='ufactory')
    limited = LaunchConfiguration('limited', default=True)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    baud_checkset = LaunchConfiguration('baud_checkset', default=True)
    default_gripper_baud = LaunchConfiguration('default_gripper_baud', default=2000000)
    load_planning_scene = LaunchConfiguration('load_planning_scene', default='true')

    launch_args = {
        'robot_ip': robot_ip,
        'report_type': report_type,
        'baud_checkset': baud_checkset,
        'default_gripper_baud': default_gripper_baud,
        'dof': '6',
        'prefix': prefix,
        'hw_ns': hw_ns,
        'limited': limited,
        'effort_control': effort_control,
        'velocity_control': velocity_control,
        'add_gripper': add_gripper,
        'add_vacuum_gripper': add_vacuum_gripper,
        'robot_type': 'uf850',
        'ros2_control_plugin': 'uf_robot_hardware/UFRobotSystemHardware',
    }
    launch_args.update(eef_params)

    robot_moveit_servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_servo'), 'launch', '_robot_moveit_servo_realmove.launch.py'])),
        launch_arguments=launch_args.items(),
    )

    planning_scene_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_servo'), 'launch', 'xarm_moveit_servo_scene_pilotus.launch.py'])),
        condition=IfCondition(load_planning_scene),
    )

    return [robot_moveit_servo_launch, planning_scene_launch]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', description='Robot IP address'),
        DeclareLaunchArgument(
            'load_planning_scene',
            default_value='true',
            description='Whether to load the planning scene (default: true).'
        ),
        DeclareLaunchArgument(
            'eef_config_file',
            default_value='',
            description='Path to uf850_eef_params.yaml. If empty, uses package default.'
        ),
        OpaqueFunction(function=_launch_setup),
    ]) 