#!/usr/bin/env python3

# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():

    remappable_topics = [
        DeclareLaunchArgument("ego_data_topic", default_value="~/ego_data", description="ego data topic"),
        DeclareLaunchArgument("route_topic", default_value="~/route", description="planned route topic"),
        DeclareLaunchArgument("goal_pose_topic", default_value="/goal_pose", description="goal pose topic"),
    ]

    args = [
        DeclareLaunchArgument("name", default_value="lanelet2_route_planning", description="node name"),
        DeclareLaunchArgument("namespace", default_value="", description="node namespace"),
        DeclareLaunchArgument(
            "params",
            default_value=os.path.join(get_package_share_directory("lanelet2_route_planning"), "config", "params.yml"),
            description="path to parameter file",
        ),
        DeclareLaunchArgument(
            "log_level", default_value="info", description="ROS logging level (debug, info, warn, error, fatal)"
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="use simulation clock"),
        *remappable_topics,
    ]

    nodes = [
        Node(
            package="lanelet2_route_planning",
            executable="lanelet2_route_planning",
            namespace=LaunchConfiguration("namespace"),
            name=LaunchConfiguration("name"),
            parameters=[LaunchConfiguration("params")],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            remappings=[(la.default_value[0].text, LaunchConfiguration(la.name)) for la in remappable_topics],
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="plan_route_action_client",
            executable="plan_route_action_client",
            namespace=LaunchConfiguration("namespace"),
            name="plan_route_action_client",
            parameters=[LaunchConfiguration("params")],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            remappings=[(la.default_value[0].text, LaunchConfiguration(la.name)) for la in remappable_topics],
            output="screen",
            emulate_tty=True,
        ),
    ]

    return LaunchDescription(
        [
            *args,
            SetParameter("use_sim_time", LaunchConfiguration("use_sim_time")),
            *nodes,
        ]
    )
