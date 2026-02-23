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
        results = self.model(cv_image, conf=0.5)        #Runs the YOLO algorithm. conf is how confident the model has to be to mark the point
        
        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()       #Tensor with bounding box coordinates
            center = (int((x1 + x2) / 2), int((y1 + y2) / 2))       #Center is average position of x and y
            
            cv2.circle(cv_image, center, 5, (0, 0, 255), -1)        #Draws circle on the image at the detected center point.
        
        cv2.imshow('YOLO deteksjon', cv_image)
        cv2.waitKey(1)

    #Class that shows what the YOLO model detects. Used when checking wether the YOLO model detects what it should.
    def process_image_yolo_detection_debugging(self, msg):
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