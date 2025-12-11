#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription

# Robot description from URDF
urdf_path = os.path.join(get_package_share_directory("maddrone"), "urdf", "maddrone.urdf")
with open(urdf_path, "r") as f:
    robot_description = f.read()

maddrone_description_share_dir = get_package_share_directory("maddrone")


def qvio_ned_to_enu():
    return Node(
        package="maddrone",
        executable="qvio_ned_to_enu",
        name="qvio_ned_to_enu",
        output="screen",
        parameters=[{
            'input_odom_topic': '/qvio/odom',
            'output_odom_topic': '/qvio_enu',
            'publish_tf': False,  # DON'T publish TF - let EKF handle it
        }]
    )


def robot_state_publisher_node():
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            {"robot_description": robot_description},
        ],
    )


def joint_state_publisher_node():
    return Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            {"robot_description": robot_description},
        ],
    )

def ekf_odom():
    return Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[maddrone_description_share_dir + "/config/ekf.yaml"],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
        ]
    )

def scanmatcher_node():
    return Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        name='scanmatcher',
        remappings=[
            ('/input_cloud', '/tof_pc'),
            ('/odom', '/odometry/filtered'),  # Use EKF filtered odometry
            ('imu', '/imu_apps'),
        ],
        parameters=[{
            'registration_method': 'NDT',
            'ndt_resolution': 0.2,  # Increase from 0.1 for more stability
            'ndt_num_threads': 4,
            'ndt_max_iterations': 30,
            'trans_for_mapupdate': 0.5,  # Update map less frequently
            'voxel_leaf_size': 0.2,  # Match ndt_resolution
            'scan_min_range': 0.3,  # Filter out very close points
            'scan_max_range': 20.0,
            'use_odom': True,
            'use_imu': False,  # Keep disabled if IMU quality is uncertain
            'set_initial_pose': True,
            'initial_pose_x': 0.0,
            'initial_pose_y': 0.0,
            'initial_pose_z': 0.0,
            'initial_pose_qx': 0.0,
            'initial_pose_qy': 0.0,
            'initial_pose_qz': 0.0,
            'initial_pose_qw': 1.0,
            'debug_flag': True,
        }],
        output='screen'
    )
    

def graph_based_slam_node():
    return Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        name='graph_based_slam',
        parameters=[{
            'registration_method': 'NDT',
            'ndt_resolution': 0.2,  # Match scanmatcher
            'ndt_num_threads': 4,
            'trans_for_mapupdate': 0.5,
            'voxel_leaf_size': 0.2,  # Match ndt_resolution
            'loop_detection_period': 3000,  # Check less frequently (3s)
            'threshold_loop_closure_score': 2.0,  # Be more strict (higher = stricter)
            'distance_loop_closure': 8.0,
            'range_of_searching_loop_closure': 10.0,
            'search_submap_num': 3,
            'num_adjacent_pose_constraints': 5,  # More constraints for stability
        }],
        output='screen'
    )


def rviz_node():
    return Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
    )


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(robot_state_publisher_node())
    ld.add_action(joint_state_publisher_node())
    ld.add_action(qvio_ned_to_enu())
    ld.add_action(ekf_odom())  # EKF will publish odom->base_link
    ld.add_action(scanmatcher_node())
    ld.add_action(graph_based_slam_node())
    ld.add_action(rviz_node())
    return ld