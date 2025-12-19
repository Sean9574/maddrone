#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
from launch_ros.actions import Node

from launch import LaunchDescription

# Robot description from URDF
urdf_path = os.path.join(get_package_share_directory("maddrone"), "urdf", "maddrone.urdf")
with open(urdf_path, "r") as f:
    robot_description = f.read()

maddrone_share = get_package_share_directory("maddrone")


def qvio_ned_to_enu():
    return Node(
        package="maddrone",
        executable="qvio_ned_to_enu",
        name="qvio_ned_to_enu",
        output="screen",
        parameters=[{
            "input_odom_topic": "/qvio/odom",
            "output_odom_topic": "/qvio_enu",
            "publish_tf": False,  # keep False unless you intentionally want qvio to own odom->base_link TF
        }],
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
    # NOTE: Fix TF timing in ekf.yaml (recommended):
    # - increase frequency (e.g., 50)
    # - consider transform_time_offset (e.g., 0.05)
    # See robot_localization docs for transform_time_offset. 
    return Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(maddrone_share, "config", "ekf.yaml")],
        remappings=[
            ("odometry/filtered", "/odometry/filtered"),
        ],
    )


def scanmatcher_node():
    return Node(
        package="scanmatcher",
        executable="scanmatcher_node",
        name="scanmatcher",
        output="screen",
        remappings=[
            ("input_cloud", "/tof_pc"),
            ("imu", "/imu_apps"),
        ],
        parameters=[{
            # GICP often works better for dense, noisy ToF data
            "registration_method": "GICP",
            
            # If you prefer NDT, use reasonable resolution (meters, not mm!)
            # "registration_method": "NDT",
            # "ndt_resolution": 0.5,  # 0.3-1.0 for indoor, NOT 0.001!
            
            "ndt_num_threads": 0,  # 0 = use all available cores
            
            # GICP settings (if using GICP)
            "gicp_corr_dist_threshold": 1.0,  # correspondence distance threshold
            
            # Map update frequency - balance between accuracy and compute
            "trans_for_mapupdate": 0.3,  # update every 30cm of movement
            "map_publish_period": 2.0,   # publish map every 2s for debug
            
            # Voxel filtering - input should be MORE aggressive than map
            "vg_size_for_input": 0.05,   # 5cm voxels for input (aggressive filtering)
            "vg_size_for_map": 0.02,     # 2cm voxels for map (keep detail)
            
            # ToF sensor realistic range limits
            "scan_min_range": 0.2,   # ToF min range
            "scan_max_range": 6.0,   # ToF effective max (not spec max)
            
            # Use more clouds for registration to handle limited FoV
            "num_targeted_cloud": 15,  # default is 10, increase for narrow FoV sensors
            
            # CRITICAL: Enable IMU for drone!
            "use_imu": True,
            "scan_period": 0.033,  # ~30Hz ToF, adjust to your actual rate
            
            # Use VOXL's VIO odometry as prior
            "use_odom": True,
            
            "publish_tf": True,
            "set_initial_pose": True,
            "initial_pose_x": 0.0,
            "initial_pose_y": 0.0,
            "initial_pose_z": 0.0,
            "initial_pose_qx": 0.0,
            "initial_pose_qy": 0.0,
            "initial_pose_qz": 0.0,
            "initial_pose_qw": 1.0,
            
            "debug_flag": True,
        }],
    )


def graph_based_slam_node():
    return Node(
        package="graph_based_slam",
        executable="graph_based_slam_node",
        name="graph_based_slam",
        output="screen",
        parameters=[{
            # Match frontend method
            "registration_method": "GICP",
            
            # If using NDT in frontend, use same here
            # "registration_method": "NDT",
            # "ndt_resolution": 0.5,
            
            "ndt_num_threads": 0,
            
            "voxel_leaf_size": 0.05,  # match or slightly larger than input voxel
            
            # Loop closure - be more lenient given ToF limitations
            "loop_detection_period": 2000,      # check every 2s
            "threshold_loop_closure_score": 1.0, # lower = stricter matching
            "distance_loop_closure": 5.0,        # min travel before revisit counts
            "range_of_searching_loop_closure": 6.0,  # within ToF range
            "search_submap_num": 4,              # more submaps for narrow FoV
            "num_adjacent_pose_constraints": 5,
            
            # Careful with this - can cause lag
            "use_save_map_in_loop": False,  # save manually instead
        }],
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

    ld.add_action(qvio_ned_to_enu())
    ld.add_action(ekf_odom())

    # Delay scanmatcher + backend a bit so TF buffers exist (reduces empty-frame TF errors)
    # ld.add_action(TimerAction(period=2.0, actions=[scanmatcher_node()]))
    # ld.add_action(TimerAction(period=2.5, actions=[graph_based_slam_node()]))
    ld.add_action(rgb_compression_node())
    ld.add_action(rviz_node())
    return ld
