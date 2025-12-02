#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation


class QVIONedToEnuTransform(Node):
    """
    Transform QVIO odometry from NED (North-East-Down) to ENU (East-North-Up) coordinate frame.
    
    NED is the aerospace convention used by VOXL QVIO
    ENU is the robotics convention used by most ROS packages
    """
    
    def __init__(self):
        super().__init__('qvio_ned_to_enu')
        
        # Declare parameters
        self.declare_parameter('input_odom_topic', '/qvio/odom')
        self.declare_parameter('output_odom_topic', '/qvio_enu')
        self.declare_parameter('output_frame_id', 'odom')
        self.declare_parameter('output_child_frame_id', 'base_link')
        
        # Get parameters
        input_topic = self.get_parameter('input_odom_topic').value
        output_topic = self.get_parameter('output_odom_topic').value
        self.output_frame_id = self.get_parameter('output_frame_id').value
        self.output_child_frame_id = self.get_parameter('output_child_frame_id').value
        
        # Create QoS profile with BEST_EFFORT to match ekf_filter_node
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Rotation matrix from NED to ENU
        # ENU has X=East, Y=North, Z=Up
        # NED has X=North, Y=East, Z=Down
        # Transformation: [E, N, U] = [Y, X, -Z] from NED
        # With corrected signs for orientation
        self.R_ned_to_enu = np.array([
            [0,  1,  0],   # ENU_x = NED_y (East)
            [1,  0,  0],   # ENU_y = NED_x (North)  
            [0,  0, -1]    # ENU_z = -NED_z (Up)
        ])
        
        # Additional rotation to fix orientation: 180 degree rotation around Z axis
        self.R_fix = np.array([
            [-1,  0,  0],
            [ 0, -1,  0],
            [ 0,  0,  1]
        ])
        
        # Combined transformation
        self.R_combined = self.R_fix @ self.R_ned_to_enu
        
        # Create subscriber with BEST_EFFORT
        self.odom_sub = self.create_subscription(
            Odometry,
            input_topic,
            self.odom_callback,
            qos_profile
        )
        
        # Create publisher with BEST_EFFORT
        self.odom_pub = self.create_publisher(
            Odometry,
            output_topic,
            qos_profile
        )
        
        self.get_logger().info('QVIO NED to ENU Transform Node Started')
        self.get_logger().info(f'Input topic: {input_topic}')
        self.get_logger().info(f'Output topic: {output_topic}')
        self.get_logger().info(f'Output frame: {self.output_frame_id}')
        
    def transform_position(self, ned_pos):
        """Transform position from NED to ENU"""
        ned_array = np.array([ned_pos[0], ned_pos[1], ned_pos[2]])
        enu_array = self.R_combined @ ned_array
        return enu_array.tolist()
    
    def transform_quaternion(self, ned_quat):
        """
        Transform quaternion from NED to ENU frame
        """
        # Convert NED quaternion to rotation matrix
        r_ned = Rotation.from_quat([ned_quat[0], ned_quat[1], ned_quat[2], ned_quat[3]])
        R_body_ned = r_ned.as_matrix()
        
        # Transform to ENU frame with fix: R_body_enu = R_combined * R_body_ned * R_combined^T
        R_body_enu = self.R_combined @ R_body_ned @ self.R_combined.T
        
        # Convert back to quaternion
        r_enu = Rotation.from_matrix(R_body_enu)
        enu_quat = r_enu.as_quat()  # Returns [x, y, z, w]
        
        return enu_quat.tolist()
    
    def transform_velocity(self, ned_vel):
        """Transform velocity from NED to ENU"""
        ned_array = np.array([ned_vel[0], ned_vel[1], ned_vel[2]])
        enu_array = self.R_combined @ ned_array
        return enu_array.tolist()
    
    def odom_callback(self, msg):
        """
        Callback to transform odometry from NED to ENU
        """
        # Create new odometry message
        odom_enu = Odometry()
        
        # Copy and update header
        odom_enu.header = msg.header
        odom_enu.header.frame_id = self.output_frame_id
        odom_enu.child_frame_id = self.output_child_frame_id
        
        # Transform position
        ned_pos = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]
        enu_pos = self.transform_position(ned_pos)
        
        odom_enu.pose.pose.position.x = enu_pos[0]
        odom_enu.pose.pose.position.y = enu_pos[1]
        odom_enu.pose.pose.position.z = enu_pos[2]
        
        # Transform orientation (quaternion)
        ned_quat = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        enu_quat = self.transform_quaternion(ned_quat)
        
        odom_enu.pose.pose.orientation.x = enu_quat[0]
        odom_enu.pose.pose.orientation.y = enu_quat[1]
        odom_enu.pose.pose.orientation.z = enu_quat[2]
        odom_enu.pose.pose.orientation.w = enu_quat[3]
        
        # Transform linear velocity
        ned_lin_vel = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ]
        enu_lin_vel = self.transform_velocity(ned_lin_vel)
        
        odom_enu.twist.twist.linear.x = enu_lin_vel[0]
        odom_enu.twist.twist.linear.y = enu_lin_vel[1]
        odom_enu.twist.twist.linear.z = enu_lin_vel[2]
        
        # Transform angular velocity
        ned_ang_vel = [
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ]
        enu_ang_vel = self.transform_velocity(ned_ang_vel)
        
        odom_enu.twist.twist.angular.x = enu_ang_vel[0]
        odom_enu.twist.twist.angular.y = enu_ang_vel[1]
        odom_enu.twist.twist.angular.z = enu_ang_vel[2]
        
        # Copy covariances (for a more rigorous transformation, rotate covariance matrices)
        odom_enu.pose.covariance = msg.pose.covariance
        odom_enu.twist.covariance = msg.twist.covariance
        
        # Publish transformed odometry
        self.odom_pub.publish(odom_enu)


def main(args=None):
    rclpy.init(args=args)
    node = QVIONedToEnuTransform()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()