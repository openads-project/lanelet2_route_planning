#!/usr/bin/env python

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter


def generate_launch_description():

    params_arg = DeclareLaunchArgument('params', default_value='params.yml')
    config = PathJoinSubstitution([
        get_package_share_directory("lanelet2_route_planning"), "config",
        LaunchConfiguration('params')
    ])

    node_name_arg = DeclareLaunchArgument('node_name', default_value='route_planning')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='False')
    ego_data_topic_arg = DeclareLaunchArgument('ego_data_topic', default_value='~/ego_data')
    goal_pose_topic_arg = DeclareLaunchArgument('goal_pose_topic', default_value='/goal_pose')

    route_planning_node = Node(
        package="lanelet2_route_planning",
        executable="global_planner_node",
        name=LaunchConfiguration('node_name'),
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[config],
        remappings=[('~/ego_data', LaunchConfiguration('ego_data_topic'))],
    )

    route_planning_action_client = Node(
        package="global_maneuver_action_client",
        executable="global_maneuver_action_client_node",
        name="route_planning_action_client",
        namespace="",
        output="screen",
        parameters=[config],
        remappings=[('~/goal_pose', LaunchConfiguration('goal_pose_topic'))],
    )

    return LaunchDescription([
        params_arg,
        node_name_arg,
        use_sim_time_arg,
        ego_data_topic_arg,
        goal_pose_topic_arg,
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        route_planning_node,
        route_planning_action_client
    ])
