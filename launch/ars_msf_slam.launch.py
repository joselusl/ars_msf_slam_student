#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Define the arguments
    screen_arg = DeclareLaunchArgument(
        'screen', default_value='screen',
        description='Output setting for the nodes'
    )

    ars_msf_slam_node_name_arg = DeclareLaunchArgument(
        'ars_msf_slam_node_name', default_value='ars_msf_slam_node',
        description='Name of the node'
    )

    ars_msf_slam_yaml_file_arg=DeclareLaunchArgument(
      'config_param_msf_slam_yaml_file',
      default_value=PathJoinSubstitution(['config_msf_slam.yaml']), 
      description='Path to the config_param_msf_slam_yaml_file'
    )


    meas_robot_position_topic_arg = DeclareLaunchArgument(
        'meas_robot_position_topic', default_value='/meas_robot_position',
        description='Topic meas_robot_position'
    )

    meas_robot_attitude_topic_arg = DeclareLaunchArgument(
        'meas_robot_attitude_topic', default_value='/meas_robot_attitude',
        description='Topic meas_robot_attitude'
    )

    meas_robot_velocity_robot_topic_arg = DeclareLaunchArgument(
        'meas_robot_velocity_robot_topic', default_value='/meas_robot_velocity_robot',
        description='Topic meas_robot_velocity_robot'
    )

    meas_obstacles_detected_robot_topic_arg = DeclareLaunchArgument(
        'meas_obstacles_detected_robot_topic', default_value='/obstacles_detected_robot',
        description='Topic meas_obstacles_detected_robot'
    )

    estim_robot_pose_topic_arg = DeclareLaunchArgument(
        'estim_robot_pose_topic', default_value='/estim_robot_pose',
        description='Topic estim_robot_pose'
    )

    estim_robot_pose_cov_topic_arg = DeclareLaunchArgument(
        'estim_robot_pose_cov_topic', default_value='/estim_robot_pose_cov',
        description='Topic estim_robot_pose_cov'
    )

    estim_robot_velocity_robot_topic_arg = DeclareLaunchArgument(
        'estim_robot_velocity_robot_topic', default_value='/estim_robot_velocity_robot',
        description='Topic estim_robot_velocity_robot'
    )

    estim_robot_velocity_robot_cov_topic_arg = DeclareLaunchArgument(
        'estim_robot_velocity_robot_cov_topic', default_value='/estim_robot_velocity_robot_cov',
        description='Topic estim_robot_velocity_robot_cov'
    )

    estim_robot_velocity_world_topic_arg = DeclareLaunchArgument(
        'estim_robot_velocity_world_topic', default_value='/estim_robot_velocity_world',
        description='Topic estim_robot_velocity_world'
    )

    estim_robot_velocity_world_cov_topic_arg = DeclareLaunchArgument(
        'estim_robot_velocity_world_cov_topic', default_value='/estim_robot_velocity_world_cov',
        description='Topic estim_robot_velocity_world_cov'
    )

    estim_map_world_topic_arg = DeclareLaunchArgument(
        'estim_map_world_topic', default_value='/estim_map_world',
        description='Topic estim_map_world'
    )


    # Get the launch configuration for parameters
    ars_msf_slam_conf_yaml_file = PathJoinSubstitution([FindPackageShare('ars_msf_slam'), 'config', LaunchConfiguration('config_param_msf_slam_yaml_file')])
    

    # Define the nodes
    ars_msf_slam_node = Node(
        package='ars_msf_slam',
        executable='ars_msf_slam_ros_node',
        name=LaunchConfiguration('ars_msf_slam_node_name'),
        output=LaunchConfiguration('screen'),
        parameters=[{'config_param_msf_slam_yaml_file': ars_msf_slam_conf_yaml_file}],
        remappings=[
          ('meas_robot_position', LaunchConfiguration('meas_robot_position_topic')),
          ('meas_robot_attitude', LaunchConfiguration('meas_robot_attitude_topic')),
          ('meas_robot_velocity_robot', LaunchConfiguration('meas_robot_velocity_robot_topic')),
          ('obstacles_detected_robot', LaunchConfiguration('meas_obstacles_detected_robot_topic')),
          ('estim_robot_pose', LaunchConfiguration('estim_robot_pose_topic')),
          ('estim_robot_pose_cov', LaunchConfiguration('estim_robot_pose_cov_topic')),
          ('estim_robot_velocity_robot', LaunchConfiguration('estim_robot_velocity_robot_topic')),
          ('estim_robot_velocity_robot_cov', LaunchConfiguration('estim_robot_velocity_robot_cov_topic')),
          ('estim_robot_velocity_world', LaunchConfiguration('estim_robot_velocity_world_topic')),
          ('estim_robot_velocity_world_cov', LaunchConfiguration('estim_robot_velocity_world_cov_topic')),
          ('estim_map_world', LaunchConfiguration('estim_map_world_topic')),
        ]
    )


    return LaunchDescription([
        screen_arg,
        ars_msf_slam_node_name_arg,
        ars_msf_slam_yaml_file_arg,
        meas_robot_position_topic_arg,
        meas_robot_attitude_topic_arg,
        meas_robot_velocity_robot_topic_arg,
        meas_obstacles_detected_robot_topic_arg,
        estim_robot_pose_topic_arg,
        estim_robot_pose_cov_topic_arg,
        estim_robot_velocity_robot_topic_arg,
        estim_robot_velocity_robot_cov_topic_arg,
        estim_robot_velocity_world_topic_arg,
        estim_robot_velocity_world_cov_topic_arg,
        estim_map_world_topic_arg,
        ars_msf_slam_node,
    ])
