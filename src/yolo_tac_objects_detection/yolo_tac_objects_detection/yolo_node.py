import rclpy
from rclpy.node import Node
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
        self.sub_left = self.create_subscription(
            Image,
            '/gbr/cam_left/image_color',
            self.left_camera_callback,
            10)
    
        self.sub_right = self.create_subscription(
            Image,
            '/gbr/cam_right/image_color',
            self.right_camera_callback,
            10)
        
        self.sub_info = self.create_subscription(       #Only one info subscription needed, since both cameras are the same. Only needed for simulator
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
    
    def camera_info_callback(self, msg):
        self.focal_length = msg.k[0]

    def left_camera_callback(self, msg):
        self.process_image(msg, True)
    
    def right_camera_callback(self, msg):
        self.process_image(msg, False)

    def process_image(self, msg, left):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(cv_image, conf=0.5)        #Runs the YOLO algorithm. conf is how confident the model has to be to mark the point
        
        for i in range(len(results[0].boxes)):
            box = results[0].boxes[i]
            x1, y1, x2, y2 = box.xyxy[0].tolist()       #Tensor with bounding box coordinates
            center = (int((x1 + x2) / 2), int((y1 + y2) / 2))       #Center is average position of x and y
            
            cv2.circle(cv_image, center, 5, (0, 0, 255), -1)        #Draws circle on the image at the detected center point.

            if left:
                self.left_classes.append(box.cls[0])
                self.left_pos_x.append(center[0])
                self.left_pos_y.append(center[1])
            else:
                self.right_classes.append(box.cls[0])
                self.right_pos_x.append(center[0])
                self.right_pos_y.append(center[1])

        self.calculate_depth(cv_image, left)

    def calculate_depth(self, image, left):
        if not self.left_classes or not self.right_classes or self.focal_length is None:     #If nothing detected, nothing to calculate depth on
            if left:
                cv2.imshow('YOLO deteksjon med dybde', image)
                cv2.waitKey(1)
            return

        class_to_look_for = 3       #Thinks this is valve
        #Zips information to create one iterable list
        left_objects = list(zip(self.left_classes, self.left_pos_x, self.left_pos_y))
        right_objects = list(zip(self.right_classes, self.right_pos_x, self.right_pos_y))

        left_objects.sort(key=lambda obj: (obj[0], obj[2]))     #Sorts classes, first based on class, and same class is sorted on y position
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
            
            #Here it is needed to take the average of the left, and right camera instead of just using the left to improve accuracy.
            if left_class == class_to_look_for:
                publish_msg = Float64MultiArray()
                left_x = (left_x - 160) * depth / self.focal_length     #Equation from https://www.reddit.com/r/opencv/comments/1enuoo0/question_project_convert_pixel_to_meter_real/
                left_y = (left_y - 160) * depth / self.focal_length
                position_class = [left_x, left_y, depth, class_to_look_for]
                publish_msg.data = position_class
                self.distance_publisher.publish(publish_msg)

        if left:
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