import rclpy
from rclpy.node import Node
from message_filters import Subscriber, TimeSynchronizer, ApproximateTimeSynchronizer
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import torch

torch.cuda.is_available = lambda: False         #Set true if gpu is available, works fine without.

class yolo_node(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        #todo fix absolute file path
        self.model = YOLO('/home/gud/Skole/baesjlort/src/yolo_tac_objects_detection/yolo_tac_objects_detection/weights_yolo/weights.pt')
        self.bridge = CvBridge()
        self.right_pos_x = []
        self.right_pos_y = []
        self.right_classes = []
        self.left_pos_x = []
        self.left_pos_y = []
        self.left_classes = []
        self.baseline = 0.042
        self.focal_length = None
        self.sub_left = Subscriber(self, Image, '/gbr/cam_left/image_color')
        self.sub_right = Subscriber(self, Image, '/gbr/cam_right/image_color')
        
        self.approx_time_sync = ApproximateTimeSynchronizer([self.sub_left, self.sub_right], queue_size=5, slop=0.05)
        self.approx_time_sync.registerCallback(self.image_sync_callback)

        self.sub_info = self.create_subscription(       #Only one info subscription needed, since both cameras are the same. Only needed for simulator as manual calibration IRL
            CameraInfo,
            '/gbr/cam_left/camera_info',
            self.camera_info_callback,
            1
        )

        self.distance_publisher = self.create_publisher(
            Float64MultiArray,
            'distance_to_object',
            10
        )
    
    def image_sync_callback(self, msg_left, msg_right):
        self.process_image_2(msg_left, msg_right)

    def camera_info_callback(self, msg):
        self.focal_length = msg.k[0]

    def process_image_2(self, left_msg, right_msg):
        cv_image_left = self.bridge.imgmsg_to_cv2(left_msg, 'bgr8')
        cv_image_right = self.bridge.imgmsg_to_cv2(right_msg, 'bgr8')
        results_left = self.model(cv_image_left, conf=0.5)        #Runs the YOLO algorithm. conf is how confident the model has to be to mark the point
        results_right = self.model(cv_image_right, conf=0.5)
        
        for i in range(len(results_left[0].boxes)):
            box = results_left[0].boxes[i]
            x1, y1, x2, y2 = box.xyxy[0].tolist()       #Tensor with bounding box coordinates to variables
            center = (int((x1 + x2) / 2), int((y1 + y2) / 2))       #Center is average position of x and y
            
            cv2.circle(cv_image_left, center, 5, (0, 0, 255), -1)        #Draws circle on the image at the detected center point.

            self.left_classes.append(box.cls[0])
            self.left_pos_x.append(center[0])
            self.left_pos_y.append(center[1])

        for i in range(len(results_right[0].boxes)):
            box = results_right[0].boxes[i]
            x1, y1, x2, y2 = box.xyxy[0].tolist()       #Tensor with bounding box coordinates to variables
            center = (int((x1 + x2) / 2), int((y1 + y2) / 2))       #Center is average position of x and y
            
            cv2.circle(cv_image_left, center, 5, (0, 0, 255), -1)        #Draws circle on the image at the detected center point.

            self.right_classes.append(box.cls[0])
            self.right_pos_x.append(center[0])
            self.right_pos_y.append(center[1])

        self.calculate_depth(cv_image_left)

    def calculate_depth(self, image):
        if not self.left_classes or not self.right_classes or self.focal_length is None:     #If nothing detected, nothing to calculate depth on
            cv2.imshow('YOLO deteksjon med dybde', image)
            cv2.waitKey(1)
            return

        class_to_look_for = 3       #Look for valve
        #Zips information to create one iterable list
        left_objects = list(zip(self.left_classes, self.left_pos_x, self.left_pos_y))
        right_objects = list(zip(self.right_classes, self.right_pos_x, self.right_pos_y))

        left_objects.sort(key=lambda obj: (obj[0], obj[2]))     #Sorts classes, first based on class, and same class is sorted on y position (Sorting method found using ChatGPT)
        right_objects.sort(key=lambda obj: (obj[0], obj[2]))
        depth = 0

        for i in range(min(len(left_objects), len(right_objects))):
            left_class, left_x, left_y = left_objects[i]
            right_class, right_x, right_y = right_objects[i]
            
            if left_class == right_class:
                    depth = (self.baseline * self.focal_length) / (left_x - right_x )       #Equation from https://www.youtube.com/watch?v=hUVyDabn1Mg&list=PL2zRqk16wsdoCCLpou-dGo7QQNks1Ppzo&index=6 at 10:20
                    
                    #Draw depth info on image (By ChatGPT) (for visualization)
                    cv2.putText(image, 
                            f"{depth:.2f}m", 
                            (left_x, left_y - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 
                            0.5, 
                            (0, 0, 255), 
                            2)
            
            if left_class == class_to_look_for:
                publish_msg = Float64MultiArray()
                meter_x = (((left_x) * depth / self.focal_length) + ((right_x) * depth / self.focal_length))*0.5     #Equation from https://www.reddit.com/r/opencv/comments/1enuoo0/question_project_convert_pixel_to_meter_real/ without cx part.
                meter_y = (((left_y) * depth / self.focal_length) + ((right_y) * depth / self.focal_length))*0.5 
                position_class = [meter_x, meter_y, depth, class_to_look_for]
                publish_msg.data = position_class
                self.distance_publisher.publish(publish_msg)

        cv2.imshow('YOLO deteksjon med dybde', image)
        cv2.waitKey(1)

        self.right_pos_x = []
        self.right_pos_y = []
        self.right_classes = []
        self.left_pos_x = []
        self.left_pos_y = []
        self.left_classes = []

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