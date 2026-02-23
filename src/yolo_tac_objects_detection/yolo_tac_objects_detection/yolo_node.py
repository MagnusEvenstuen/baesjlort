import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import torch

torch.cuda.is_available = lambda: False

class yolo_node(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        #todo fix absolute file path
        self.model = YOLO('/home/gud/Skole/baesjlort/src/yolo_tac_objects_detection/yolo_tac_objects_detection/weights_yolo/weights.pt')
        self.bridge = CvBridge()
        self.sub_left = self.create_subscription(
            Image,
            '/gbr/cam_left/image_color',
            self.left_camera_callback,
            10)
    
        #self.sub_right = self.create_subscription(
        #    Image,
        #    '/gbr/cam_right/image_color',
        #    self.right_camera_callback,
        #    10)
    
    def left_camera_callback(self, msg):
        self.process_image(msg)
    
    #def right_camera_callback(self, msg):
        #self.process_image(msg)

    def process_image(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(cv_image, conf=0.4)
        annotated_image = results[0].plot()

        cv2.imshow(f'YOLO deteksjon', annotated_image)
        cv2.waitKey(1)

def main():
    rclpy.init(args=None)
    node = yolo_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()