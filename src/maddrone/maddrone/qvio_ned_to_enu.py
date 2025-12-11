#!/usr/bin/env python3


import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from tf2_ros import TransformBroadcaster


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
        self.declare_parameter('body_frame_rotation_deg', 90)  # Rotation to align body frame (X forward)
        self.declare_parameter('publish_tf', True)
        self.publish_tf = self.get_parameter('publish_tf').value
        # Get parameters
        input_topic = self.get_parameter('input_odom_topic').value
        output_topic = self.get_parameter('output_odom_topic').value
        self.output_frame_id = self.get_parameter('output_frame_id').value
        self.output_child_frame_id = self.get_parameter('output_child_frame_id').value
        body_rotation_deg = self.get_parameter('body_frame_rotation_deg').value
        
        # Create QoS profile with BEST_EFFORT to match ekf_filter_node
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Rotation matrix from NED to ENU (for WORLD FRAME - position/orientation)
        # ENU has X=East, Y=North, Z=Up
        # NED has X=North, Y=East, Z=Down
        # Transformation: [E, N, U] = [Y, X, -Z] from NED
        self.R_ned_to_enu = np.array([
            [0,  1,  0],   # ENU_x = NED_y (East)
            [1,  0,  0],   # ENU_y = NED_x (North)  
            [0,  0, -1]    # ENU_z = -NED_z (Up)
        ])
        
        # Additional rotation to fix orientation: flip X and Y axes
        self.R_fix = np.array([
            [1,  0,  0],
            [0,  1,  0],
            [0,  0,  1]
        ])
        
        # Combined transformation (for position in world frame)
        self.R_combined = self.R_fix @ self.R_ned_to_enu
        
        # Body frame transformation (for velocities in body frame)
        # Based on observed behavior: x = x, y = -y, z = -z
        # NED body: X=forward, Y=right, Z=down
        # ENU body (with 180° yaw): X=forward, Y=left, Z=up
        self.R_body = np.array([
            [1,   0,  0],   # X unchanged (forward stays forward)
            [0,  -1,  0],   # Y flipped (right becomes left)
            [0,   0, -1]    # Z flipped (down becomes up)
        ])
        
        # Body frame alignment as quaternion rotation (applied in ENU frame AFTER transform)
        self.body_frame_rotation = Rotation.from_euler('z', body_rotation_deg, degrees=True)
        
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
        
        # Create TF broadcaster to publish odom->base_link transform
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Store initial position offset to reset to 0,0,0
        self.initial_position = None
        self.position_offset_set = False
        
        self.get_logger().info('QVIO NED to ENU Transform Node Started')
        self.get_logger().info(f'Input topic: {input_topic}')
        self.get_logger().info(f'Output topic: {output_topic}')
        self.get_logger().info(f'Output frame: {self.output_frame_id}')
        self.get_logger().info(f'Body frame rotation: {body_rotation_deg}°')
        
    def transform_position(self, ned_pos):
        """Transform position from NED to ENU (world frame transformation)"""
        ned_array = np.array([ned_pos[0], ned_pos[1], ned_pos[2]])
        enu_array = self.R_combined @ ned_array
        return enu_array.tolist()
    
    def transform_quaternion(self, ned_quat):
        """
        Transform quaternion from NED to ENU frame, then apply body frame alignment.
        
        The quaternion represents the rotation from body frame to world frame.
        We need to transform this rotation to account for the world frame change (NED -> ENU).
        """
        # Convert NED quaternion to rotation object
        r_body_to_ned = Rotation.from_quat([ned_quat[0], ned_quat[1], ned_quat[2], ned_quat[3]])
        
        # Get rotation matrix in NED frame
        R_body_ned = r_body_to_ned.as_matrix()
        
        # Transform to ENU frame using same logic as position
        # R_body_enu = R_ned_to_enu * R_body_ned * R_ned_to_enu^T
        R_body_enu = self.R_ned_to_enu @ R_body_ned @ self.R_ned_to_enu.T
        
        # Convert back to rotation object
        r_body_to_enu = Rotation.from_matrix(R_body_enu)
        
        # Apply body frame rotation parameter (typically 180° to align forward axis)
        r_final = r_body_to_enu * self.body_frame_rotation
        
        # Convert back to quaternion [x, y, z, w]
        enu_quat = r_final.as_quat()
        
        return enu_quat.tolist()
    
    def transform_body_velocity(self, ned_vel):
        """
        Transform velocity from NED body frame to ENU body frame.
        
        IMPORTANT: Twist velocities are in child_frame_id (body frame), not world frame.
        This is different from position transformation which is in world frame.
        
        Based on observed behavior: x = x, y = -y, z = -z
        NED body: X=forward, Y=right, Z=down
        ENU body: X=forward, Y=left, Z=up
        """
        ned_array = np.array([ned_vel[0], ned_vel[1], ned_vel[2]])
        enu_array = self.R_body @ ned_array
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
        
        # Transform position (WORLD FRAME)
        ned_pos = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]
        enu_pos = self.transform_position(ned_pos)
        
        # Store first position as offset and subtract from all subsequent positions
        if not self.position_offset_set:
            self.initial_position = enu_pos.copy()
            self.position_offset_set = True
            self.get_logger().info(f'Initial position offset: {self.initial_position}')
        
        # Apply offset to center at 0,0,0
        odom_enu.pose.pose.position.x = enu_pos[0] - self.initial_position[0]
        odom_enu.pose.pose.position.y = enu_pos[1] - self.initial_position[1]
        odom_enu.pose.pose.position.z = enu_pos[2] - self.initial_position[2]
        
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
        
        # Transform linear velocity (BODY FRAME - not world frame!)
        ned_lin_vel = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ]
        enu_lin_vel = self.transform_body_velocity(ned_lin_vel)
        
        odom_enu.twist.twist.linear.x = enu_lin_vel[0]
        odom_enu.twist.twist.linear.y = enu_lin_vel[1]
        odom_enu.twist.twist.linear.z = enu_lin_vel[2]
        
        # Transform angular velocity (BODY FRAME - not world frame!)
        ned_ang_vel = [
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ]
        enu_ang_vel = self.transform_body_velocity(ned_ang_vel)
        
        odom_enu.twist.twist.angular.x = enu_ang_vel[0]
        odom_enu.twist.twist.angular.y = enu_ang_vel[1]
        odom_enu.twist.twist.angular.z = enu_ang_vel[2]
        
        # Copy covariances (for a more rigorous transformation, rotate covariance matrices)
        odom_enu.pose.covariance = msg.pose.covariance
        odom_enu.twist.covariance = msg.twist.covariance
        
        # Publish transformed odometry
        self.odom_pub.publish(odom_enu)
        
        # Broadcast TF transform from odom to base_link
        transform = TransformStamped()
        transform.header.stamp = odom_enu.header.stamp
        transform.header.frame_id = self.output_frame_id
        transform.child_frame_id = self.output_child_frame_id
        
        # Copy position
        transform.transform.translation.x = odom_enu.pose.pose.position.x
        transform.transform.translation.y = odom_enu.pose.pose.position.y
        transform.transform.translation.z = odom_enu.pose.pose.position.z
        
        # Copy orientation
        transform.transform.rotation.x = odom_enu.pose.pose.orientation.x
        transform.transform.rotation.y = odom_enu.pose.pose.orientation.y
        transform.transform.rotation.z = odom_enu.pose.pose.orientation.z
        transform.transform.rotation.w = odom_enu.pose.pose.orientation.w
        
        # Broadcast the transform
        if self.publish_tf:
    # Broadcast the transform
            self.tf_broadcaster.sendTransform(transform)



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
