#!/usr/bin/env python

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, SetParameter


def generate_launch_description():
    node_name_default = 'point2maneuver'
    node_name_arg = DeclareLaunchArgument('node_name',
                                          default_value=node_name_default)
    
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time_arg', default_value='False')

    node = LifecycleNode(
        package="point2maneuver",
        executable="point2maneuver_node",
        name=LaunchConfiguration('node_name'),
        namespace="",
        output="screen",
        emulate_tty=True,
    )

    node_group = GroupAction(actions=[
        SetParameter(name='use_sim_time',
                     value=LaunchConfiguration('use_sim_time_arg'),
                     condition=LaunchConfigurationNotEquals(
                         'use_sim_time_arg', "None")), node
    ])

    return LaunchDescription([
        node_name_arg,
        use_sim_time_arg,
        node_group
    ])
