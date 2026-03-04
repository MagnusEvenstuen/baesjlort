import rclpy
from rclpy.node import Node
from message_filters import Subscriber, TimeSynchronizer, ApproximateTimeSynchronizer
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64MultiArray, Int32
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import torch
import numpy as np

torch.cuda.is_available = lambda: False         #Set true if gpu is available, works fine without.

class yolo_node(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        #todo fix absolute file path
        self.model = YOLO('src/yolo_tac_objects_detection/yolo_tac_objects_detection/weights_yolo/bestYOLO11.pt')
        self.bridge = CvBridge()
        self.left_classes = []
        self.baseline = 0.042
        self.focal_length = None
        self.cx = None
        self.cy = None
        self.sub_left = Subscriber(self, Image, '/gbr/cam_left/image_color')
        self.sub_right = Subscriber(self, Image, '/gbr/cam_right/image_color')
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.parameters)
        self.clahe = cv2.createCLAHE(clipLimit=1.0)

        self.sift = cv2.SIFT_create(nfeatures=20)
        self.matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)

        self.approx_time_sync = ApproximateTimeSynchronizer([self.sub_left, self.sub_right], queue_size=1, slop=0.05)       #Only use last image
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

        self.aruco_id_publisher = self.create_publisher(
            Int32,
            'aruco_ids',
            10
        )
    
    def image_sync_callback(self, msg_left, msg_right):
        self.process_image(msg_left, msg_right)

    def camera_info_callback(self, msg):
        self.focal_length = msg.k[0]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def process_image(self, left_msg, right_msg):
        right_pos_x = [[], []]
        right_pos_y = [[], []]
        right_classes = []
        left_pos_x = [[], []]
        left_pos_y = [[], []]
        left_classes = []
        left_boxes = []
        right_boxes = []

        cv_image_left = self.bridge.imgmsg_to_cv2(left_msg, 'bgr8')
        cv_image_right = self.bridge.imgmsg_to_cv2(right_msg, 'bgr8')
        results_left = self.model(cv_image_left, conf=0.5)        #Runs the YOLO algorithm. conf is how confident the model has to be to mark the point
        results_right = self.model(cv_image_right, conf=0.5)
        
        self.save_bounding_box(results_left, left_classes, left_pos_x, left_pos_y, left_boxes, cv_image_left, cv_image_left)
        self.save_bounding_box(results_right, right_classes, right_pos_x, right_pos_y, right_boxes, cv_image_right, cv_image_left)

        self.calculate_depth(cv_image_left, left_classes, left_pos_x, left_pos_y, left_boxes, right_classes, right_pos_x, right_pos_y, right_boxes)

    def save_bounding_box(self, results, classes, pos_x, pos_y, boxes, image_to_use, image_to_draw_on):
        for i in range(len(results[0].boxes)):
            box = results[0].boxes[i]
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
            cv2.circle(image_to_draw_on, center, 5, (0, 0, 255), -1)

            classes.append(box.cls[0])
            pos_x[0].append(int(x1))
            pos_x[1].append(int(x2))
            pos_y[0].append(int(y1))
            pos_y[1].append(int(y2))
            boxes.append(image_to_use[int(y1):int(y2), int(x1):int(x2)])

    def calculate_depth(self, image, left_classes, left_pos_x, left_pos_y, left_boxes, right_classes, right_pos_x, right_pos_y, right_boxes):
        if not left_classes or not right_classes or self.focal_length is None:     #If nothing detected, nothing to calculate depth on
            cv2.imshow('YOLO deteksjon med dybde', image)
            cv2.waitKey(1)
            return

        class_to_look_for = 3       #Look for valve (object number 3)
        #Zips information to create one iterable list (those 2 liens are generated using ChatGPT)
        left_objects = [(class_number, (left_pos_x[0][j], left_pos_x[1][j]), (left_pos_y[0][j], left_pos_y[1][j])) 
                        for j, class_number in enumerate(left_classes)]
        right_objects = [(class_number, (right_pos_x[0][j], right_pos_x[1][j]), (right_pos_y[0][j], right_pos_y[1][j])) 
                         for j, class_number in enumerate(right_classes)]

        left_objects.sort(key=lambda obj: (obj[0], obj[2]))     #Sorts classes, first based on class, and same class is sorted on y position (Sorting method found using ChatGPT)
        right_objects.sort(key=lambda obj: (obj[0], obj[2]))
        depth = 0

        for i in range(min(len(left_objects), len(right_objects))):
            left_class, left_x, left_y = left_objects[i]
            right_class, right_x, right_y = right_objects[i]
            
            if left_class == right_class:
                    #Information about how to use orb gathered from https://www.geeksforgeeks.org/python/feature-matching-using-orb-algorithm-in-python-opencv/, later changed to SIFT, but is the same
                    depth = self.orb_calculate_depth(left_boxes[i], right_boxes[i], left_x[0], right_x[0])
                    if depth is not None:
                        #Draw depth info on image (By ChatGPT) (for visualization)
                        center_x = (left_x[0] + left_x[1]) // 2
                        center_y = (left_y[0] + left_y[1]) // 2
                        cv2.putText(image, 
                                f"{depth:.2f}m", 
                                (center_x, center_y), 
                                cv2.FONT_HERSHEY_SIMPLEX, 
                                0.5, 
                                (0, 0, 255), 
                                2)
            
                        self.publish_object_position((left_x[0] + left_x[1]) // 2, (right_x[0] + right_x[1]) // 2, (left_y[0] + left_y[1]) // 2, (right_y[0] + right_y[1]) // 2, depth, left_class)
                        
                        if left_class == 0:     #This isn't tested, but should work. If not working, first test should be sending in the entire image, and not just the bounding box.
                            self.publish_aruco_ids(left_boxes[i], right_boxes[i])

        cv2.imshow('YOLO deteksjon med dybde', image)
        cv2.waitKey(1)

    def orb_calculate_depth(self, left_box, right_box, left_box_left, right_box_left):
        #Does CLAHE and turns image back to BGR
        left_box = cv2.cvtColor(left_box, cv2.COLOR_BGR2Lab)
        right_box = cv2.cvtColor(right_box, cv2.COLOR_BGR2Lab)
        ll, al, bl = cv2.split(left_box)
        lr, ar, br = cv2.split(right_box)
        left_box = cv2.merge((self.clahe.apply(ll), al, bl))
        right_box = cv2.merge((self.clahe.apply(lr), ar, br))
        left_box = cv2.cvtColor(left_box, cv2.COLOR_Lab2BGR)
        right_box = cv2.cvtColor(right_box, cv2.COLOR_Lab2BGR)

        keypoints_left, descriptors_left = self.sift.detectAndCompute(cv2.cvtColor(left_box, cv2.COLOR_BGR2GRAY), None)       #Sets the colour to grey, and does orb detection on the images
        keypoints_right, descriptors_right = self.sift.detectAndCompute(cv2.cvtColor(right_box, cv2.COLOR_BGR2GRAY), None)

        if descriptors_left is None or descriptors_right is None:
            return None

        matches = self.matcher.match(descriptors_left, descriptors_right)
        point_disparity = []
        #Calculates the distance between the matches
        for match in matches:
            left_x, left_y = keypoints_left[match.queryIdx].pt
            right_x, right_y = keypoints_right[match.trainIdx].pt

            disparity = (left_x + left_box_left) - (right_x + right_box_left)
            if disparity > 0:   #Only positive disparity is valid, since left camera is to the left of the right camera
                point_disparity.append(disparity)

        if not point_disparity:
            return None
        median_disparity = np.median(point_disparity)
        depth = (self.baseline * self.focal_length)/median_disparity       #Equation from https://www.youtube.com/watch?v=hUVyDabn1Mg&list=PL2zRqk16wsdoCCLpou-dGo7QQNks1Ppzo&index=6 at 10:20
        return depth

    #This function isn't tested here, but works in a seperate python script without ROS2
    def publish_aruco_ids(self, image_left, image_right):
        _, ids_left, _ = self.detector.detectMarkers(image_left)
        _, ids_right, _ = self.detector.detectMarkers(image_right)
        if ids_left == ids_right and ids_left is not None:
            for id in ids_left:
                msg = Int32()
                msg.data = int(id[0])
                self.aruco_id_publisher.publish(msg)

    def publish_object_position(self, center_pos_left_x, center_pos_right_x, center_pos_left_y, center_pos_right_y, depth, class_number):
        if class_number == 3:
            publish_msg = Float64MultiArray()
            meter_x = ((center_pos_left_x - self.cx) * depth / self.focal_length + (center_pos_right_x - self.cx) * depth / self.focal_length)*0.5      #Equation from https://www.reddit.com/r/opencv/comments/1enuoo0/question_project_convert_pixel_to_meter_real/.
            meter_y = ((center_pos_left_y - self.cy) * depth / self.focal_length + (center_pos_right_y - self.cy) * depth / self.focal_length)*0.5
            position_class = [meter_x, meter_y, depth, class_number]
            publish_msg.data = position_class
            self.distance_publisher.publish(publish_msg)

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