#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

maddrone_share = get_package_share_directory("maddrone")

urdf_path = os.path.join(maddrone_share, "urdf", "maddrone.urdf")
with open(urdf_path, "r") as f:
    robot_description = f.read()


def robot_state_publisher_node():
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )


def joint_state_publisher_node():
    return Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
    )


def qvio_ned_to_enu_node():
    return Node(
        package="maddrone",
        executable="qvio_ned_to_enu",
        name="qvio_ned_to_enu",
        output="screen",
        parameters=[{
            "input_odom_topic": "/qvio/odom",
            "output_odom_topic": "/qvio_enu",
            "publish_tf": False,  # EKF handles odom, scanmatcher handles map->base_link
        }],
    )


def ekf_node():
    """
    EKF fuses QVIO + IMU.
    Does NOT publish TF - scanmatcher handles that.
    Output is used by scanmatcher as motion prior.
    """
    return Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(maddrone_share, "config", "ekf.yaml")],
        remappings=[("odometry/filtered", "/odometry/filtered")],
    )

def rtabmap_node():
    """
    RTAB-Map SLAM using stereo cameras + odometry.
    Publishes map→odom transform and 3D map.
    """
    return Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[os.path.join(maddrone_share, "config", "rtabmap.yaml")],
        remappings=[
            ("scan_cloud", "tof_pc"),
            ("odom", "/odometry/filtered"),
        ],
    )

def rgb_compression_node():
    return Node(
        package="maddrone",
        executable="rgb_compression",
        name="rgb_compression",
        output="screen",
        parameters=[{
            "jpeg_quality": 50,
            "resize_factor": 0.5,
            "input_topic": "/hires_small_color",
        }],
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
    ld.add_action(qvio_ned_to_enu_node())
    ld.add_action(ekf_node())
    ld.add_action(TimerAction(period=1.0, actions=[rtabmap_node()]))
    # Utilities
    ld.add_action(rgb_compression_node())
    ld.add_action(rviz_node())

    return ld