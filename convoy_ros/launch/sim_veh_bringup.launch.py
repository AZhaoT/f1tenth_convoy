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

    pure_pursuit = LaunchConfiguration('pure_pursuit')
    pure_pursuit_la = DeclareLaunchArgument('pure_pursuit', default_value='false')

    leader_gf = LaunchConfiguration('leader_gf')
    leader_gf_la = DeclareLaunchArgument('leader_gf', default_value='false')

    follower_pp = LaunchConfiguration('follower_pp')
    follower_pp_la = DeclareLaunchArgument('follower_pp', default_value='false')

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
    pure_pursuit_node = Node(
        package='convoy_ros',
        executable='pure_pursuit_node',
        name='pure_pursuit',
        parameters=[convoy_config],
        condition=IfCondition(pure_pursuit)
    )
    leader_gf_node = Node(
        package='convoy_ros',
        executable='leader_gf_node',
        name='leader_gf',
        parameters=[convoy_config],
        condition=IfCondition(leader_gf)
    )
    follower_pp_node = Node(
        package='convoy_ros',
        executable='follower_pp_node',
        name='follower_pp',
        parameters=[convoy_config],
        condition=IfCondition(follower_pp)
    )

    return LaunchDescription([
        safety_la,
        wall_follow_la,
        gap_follow_la,
        pure_pursuit_la,
        leader_gf_la,
        follower_pp_la,
        f1tenth_include,
        safety_node,
        wall_follow_node,
        gap_follow_node,
        pure_pursuit_node,
        leader_gf_node,
        follower_pp_node,
    ])

