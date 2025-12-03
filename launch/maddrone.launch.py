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
        name="ekf_filter_node",  # <<< must match ekf.yaml top-level key
        output="screen",
        parameters=[maddrone_description_share_dir + "/config/ekf.yaml"],
        respawn=True,
        respawn_delay=2.0,
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
    ld.add_action(rviz_node())
    ld.add_action(ekf_odom())
    ld.add_action(qvio_ned_to_enu())
    return ld
