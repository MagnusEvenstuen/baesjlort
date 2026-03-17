#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv_bridge
import cv2

class Decompressor(Node):
    def __init__(self):
        super().__init__('decompressor')
        self.bridge = cv_bridge.CvBridge()
        self.sub = self.create_subscription(
            CompressedImage,
            '/gbr/cam_left/image_raw/compressed',
            self.callback,
            10)
        self.pub = self.create_publisher(
            Image,
            '/gbr/cam_left/image_raw_decompressed',
            10)

    def callback(self, msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.pub.publish(img_msg)
        self.get_logger().info('Dekomprimerte og publiserte bilde', throttle_duration_sec=1)

def main(args=None):
    rclpy.init(args=args)
    node = Decompressor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()