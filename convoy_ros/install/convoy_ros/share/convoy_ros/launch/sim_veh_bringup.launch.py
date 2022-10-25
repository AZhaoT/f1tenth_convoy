#!/usr/bin/env python3

import os
import sys

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    safety = LaunchConfiguration('safety')
    safety_la = DeclareLaunchArgument('safety', default_value='false')

    wall_follow = LaunchConfiguration('wall_follow')
    wall_follow_la = DeclareLaunchArgument('wall_follow', default_value='false')

    gap_follow = LaunchConfiguration('gap_follow')
    gap_follow_la = DeclareLaunchArgument('gap_follow', default_value='false')

    # simulator launch
    f1tenth_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('f1tenth_gym_ros'),
                'launch/gym_bridge_launch.py'
            )
        )
    )

    # convoy params and nodes
    convoy_config = os.path.join(
        get_package_share_directory('convoy_ros'),
        'config',
        'sim.yaml'
    )
    safety_node = Node(
        package='convoy_ros',
        executable='safety_node',
        name='safety',
        parameters=[convoy_config],
        condition=IfCondition(safety)
    )
    wall_follow_node = Node(
        package='convoy_ros',
        executable='wall_follow_node',
        name='wall_follow',
        parameters=[convoy_config],
        condition=IfCondition(wall_follow)
    )
    gap_follow_node = Node(
        package='convoy_ros',
        executable='gap_follow_node',
        name='gap_follow',
        parameters=[convoy_config],
        condition=IfCondition(gap_follow)
    )

    return LaunchDescription([
        safety_la,
        wall_follow_la,
        gap_follow_la,
        f1tenth_include,
        safety_node,
        wall_follow_node,
        gap_follow_node,
    ])

