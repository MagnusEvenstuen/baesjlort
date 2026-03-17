import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

class JpegLSCompressor(Node):
    def __init__(self):
        super().__init__('jpegls_compressor')
        self.bridge = CvBridge()

        self.sub_left = self.create_subscription(
            Image,
            '/gbr/cam_left/image_raw',
            self.left_callback,
            10)
        self.sub_right = self.create_subscription(
            Image,
            '/gbr/cam_right/image_raw',
            self.right_callback,
            10)

        self.pub_compressed_left = self.create_publisher(
            CompressedImage,
            'jpegls_compressed_left',
            10)
        
        self.pub_compressed_right = self.create_publisher(
            CompressedImage,
            'jpegls_compressed_right',
            10)

    def left_callback(self, msg):
        self.process_image(msg, 'left')

    def right_callback(self, msg):
        self.process_image(msg, 'right')

    def process_image(self, msg, camera_side):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 100]
        _, encoded_bytes = cv2.imencode('.jpg', cv_image, encode_param)

        compressed_msg = CompressedImage()
        compressed_msg.header = msg.header
        compressed_msg.format = 'jpeg'
        compressed_msg.data = encoded_bytes.tobytes()

        if camera_side == 'left':
            self.pub_compressed_left.publish(compressed_msg)
        else:
            self.pub_compressed_right.publish(compressed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JpegLSCompressor()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()