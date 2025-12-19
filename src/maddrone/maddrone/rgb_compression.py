#!/usr/bin/env python3
"""
H264 Decoder Node for VOXL (ROS2)
Subscribes to /hires_small_encoded (H264) and publishes decoded BGR images
"""
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

try:
    import av
    HAS_AV = True
except ImportError:
    HAS_AV = False


class RGBCompressor(Node):
    def __init__(self):
        super().__init__('rgb_compression')
        
        if not HAS_AV:
            self.get_logger().error('PyAV not installed! Run: pip install av --break-system-packages')
            return
        
        self.bridge = CvBridge()
        
        # Declare parameters
        self.declare_parameter('resize_factor', 0.5)
        self.declare_parameter('input_topic', '/hires_small_encoded')
        
        self.resize_factor = self.get_parameter('resize_factor').value
        input_topic = self.get_parameter('input_topic').value
        
        # QoS - best effort for speed
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publisher - decoded image
        self.pub_decoded = self.create_publisher(
            Image,
            '~/image_raw',
            image_qos
        )
        
        # Initialize H264 codec
        self.codec = av.CodecContext.create('h264', 'r')
        
        # Subscriber
        self.sub = self.create_subscription(
            CompressedImage,
            input_topic,
            self.callback,
            image_qos
        )
        
        self.get_logger().info(
            f'H264 Decoder started - Resize: {self.resize_factor}, Input: {input_topic}'
        )

    def callback(self, msg):
        try:
            # Create packet from H264 data
            packet = av.Packet(bytes(msg.data))
            
            # Decode
            frames = self.codec.decode(packet)
            
            for frame in frames:
                # Convert to numpy BGR
                cv_image = frame.to_ndarray(format='bgr24')
                
                # Resize if needed
                if self.resize_factor != 1.0:
                    new_w = int(cv_image.shape[1] * self.resize_factor)
                    new_h = int(cv_image.shape[0] * self.resize_factor)
                    cv_image = cv2.resize(cv_image, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
                
                # Publish decoded image
                out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                out_msg.header = msg.header
                self.pub_decoded.publish(out_msg)
                
        except Exception as e:
            self.get_logger().error(f'H264 Decode error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = RGBCompressor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()