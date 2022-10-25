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

    veh_num = None
    teleop = None
    for arg in sys.argv:
        if 'vehicle_number' in str(arg):
            veh_num = int(arg.split(":=")[1])
        if 'teleop' in str(arg):
            teleop = arg.split(":=")[1]
    assert veh_num is not None, \
        "\n\nNeed to pass argument vehicle_number, i.e., " + \
        "vehicle_number:=<num>"
    assert teleop in ['ps4', 'logitech'], \
        "\n\nNeed to pass argument teleop, i.e., " + \
        "teleop:=<type> where <type> is one of 'ps4' or 'logitech'"
    print("\nAttempting to launch vehicle {}\n".format(veh_num))

    vehicle_number = LaunchConfiguration('vehicle_number')
    vehicle_number_la = DeclareLaunchArgument('vehicle_number')

    use_rviz = LaunchConfiguration('use_rviz')
    use_rviz_la = DeclareLaunchArgument('use_rviz', default_value='false')

    use_throttle_interpolator = LaunchConfiguration('use_throttle_interpolator')
    use_throttle_interpolator_la = DeclareLaunchArgument(
        'use_throttle_interpolator', default_value='false'
    )

    if teleop == "ps4":
        joy_teleop_config = os.path.join(
            get_package_share_directory('convoy_ros'),
            'config/teleop',
            'ps4_ds4_teleop.yaml'
        )
    else if teleop == "logitech":
        joy_teleop_config = os.path.join(
            get_package_share_directory('f1tenth_stack'),
            'config',
            'joy_teleop.yaml'
        )
    joy_la = DeclareLaunchArgument(
        'joy_config',
        default_value=joy_teleop_config,
        description='Descriptions for joy and joy_teleop configs'
    )

    vesc_config = os.path.join(
        get_package_share_directory('convoy_ros'),
        'config/vesc/params',
        'vesc_{}.yaml'.format(veh_num)
    )
    vesc_la = DeclareLaunchArgument(
        'vesc_config',
        default_value=vesc_config,
        description='Descriptions for vesc configs'
    )

    mux_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'mux.yaml'
    )
    mux_la = DeclareLaunchArgument(
        'mux_config',
        default_value=mux_config,
        description='Descriptions for ackermann mux configs'
    )

    # visualize if desired
    rviz_config_dir = os.path.join(
        get_package_share_directory('convoy_ros'),
        'rviz',
        'convoy_rplidar.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    # include RPLiDAR S2 launch file
    rplidar_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sllidar_ros2'),
                'launch/sllidar_s2_launch.py'
            )
        )
    )

    # joystick/teleop nodes
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[LaunchConfiguration('joy_config')]
    )
    joy_linux_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_linux',
        parameters=[LaunchConfiguration('joy_config')]
    )
    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[LaunchConfiguration('joy_config')]
    )

    # VESC nodes
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    throttle_interpolator_node = Node(
        package='f1tenth_stack',
        executable='throttle_interpolator',
        name='throttle_interpolator',
        parameters=[LaunchConfiguration('vesc_config')],
        condition=IfCondition(use_throttle_interpolator)
    )

    # Ackermann mux node
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')],
        remappings=[('ackermann_cmd_out', 'ackermann_drive')]
    )

    # TF publishers
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['0.27', '0.0', '0.11', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )

    # convoy params and nodes
    convoy_config = os.path.join(
        get_package_share_directory('convoy_ros'),
        'config',
        'real.yaml'
    )
    safety_node = Node(
        package='convoy_ros',
        executable='safety_node',
        name='safety',
        parameters=[convoy_config]
    )
    wall_follow_node = Node(
        package='convoy_ros',
        executable='wall_follow_node',
        name='wall_follow',
        parameters=[convoy_config]
    )

    return LaunchDescription([
        use_rviz_la,
        vehicle_number_la,
        use_throttle_interpolator_la,
        joy_la,
        vesc_la,
        mux_la,
        rviz_node,
        rplidar_include,
        # joy_node,
        joy_linux_node,
        joy_teleop_node,
        ackermann_to_vesc_node,
        vesc_to_odom_node,
        vesc_driver_node,
        throttle_interpolator_node,
        ackermann_mux_node,
        static_tf_node,
        safety_node,
        wall_follow_node,
    ])
