import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from message_filters import Subscriber, TimeSynchronizer, ApproximateTimeSynchronizer
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64MultiArray, Int32
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import torch
import numpy as np
import math
import csv
import numpy as np

torch.cuda.is_available = lambda: False         #Set true if gpu is available, works fine without.

class SimpleTracker:
    def __init__(self, max_missing=5):
        self.next_id = 0
        self.tracks = {}          #id = (centroid_x, centroid_y)
        self.missing = {}
        self.max_missing = max_missing

    def update(self, detections):
        if not detections:
            for time in list(self.missing.keys()):
                self.missing[time] += 1
                if self.missing[time] > self.max_missing:
                    del self.tracks[time]
                    del self.missing[time]
            return {}

        centroids = []
        for (x1, y1, x2, y2) in detections:
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            centroids.append((cx, cy))

        if not self.tracks:
            for det_idx, c in enumerate(centroids):
                time = self.next_id
                self.tracks[time] = c
                self.missing[time] = 0
                self.next_id += 1
            return {det_idx: time for det_idx, time in enumerate(self.tracks.keys())}
        
        used_tracks = set()
        used_dets = set()
        assignments = {}

        for det_idx, det_c in enumerate(centroids):
            best_tid = None
            best_dist = float('inf')
            for time, track_c in self.tracks.items():
                if time in used_tracks:
                    continue
                dx = det_c[0] - track_c[0]
                dy = det_c[1] - track_c[1]
                dist = math.hypot(dx, dy)
                if dist < best_dist and dist < 100:
                    best_dist = dist
                    best_tid = time
            if best_tid is not None:
                assignments[det_idx] = best_tid
                used_tracks.add(best_tid)
                used_dets.add(det_idx)

                self.tracks[best_tid] = det_c
                self.missing[best_tid] = 0

        for det_idx, det_c in enumerate(centroids):
            if det_idx not in used_dets:
                time = self.next_id
                self.tracks[time] = det_c
                self.missing[time] = 0
                assignments[det_idx] = time
                self.next_id += 1

        matched_ids = set(assignments.values())
        for time in list(self.tracks.keys()):
            if time not in matched_ids:
                self.missing[time] += 1
                if self.missing[time] > self.max_missing:
                    del self.tracks[time]
                    del self.missing[time]

        return assignments

class yolo_node(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        self.tracker = SimpleTracker(max_missing=5)
        self.log_file = open('objekt_posisjoner.csv', 'w', newline='')
        self.csv_logger = csv.writer(self.log_file)
        self.csv_logger.writerow(['timestamp', 'object_id', 'x_meter', 'y_meter', 'depth_meter', 'class'])
        self.model = YOLO('src/yolo_tac_objects_detection/yolo_tac_objects_detection/weights_yolo/best_subsea_test.pt')
        self.bridge = CvBridge()
        self.left_classes = []
        self.baseline = 0.0436
        self.focal_length = 538.78453       #Placeholder. Will be changed when info is rescieved.
        self.cx = 161.95527
        self.cy = 230.65832
        self.sub_left = Subscriber(self, Image, '/gbr/cam_left/image_color', qos_profile=qos_profile_sensor_data)
        self.sub_right = Subscriber(self, Image, '/gbr/cam_right/image_color', qos_profile=qos_profile_sensor_data)
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.parameters)
        self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(4, 4))
        #Parameters found through tuning, might need further tuning
        self.orb = cv2.ORB_create(
            nfeatures=500,      #Default 500, more features = more chances to match
            scaleFactor=1.1,     #Default 1.2, finer scale pyramid
            nlevels=10,          #Default 8, more pyramid levels
            edgeThreshold=10,    #Default 31, detect closer to edges
            fastThreshold=10     #Default 20, lower = detects lower-contrast corners
        )
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

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
        results_left = self.model(cv_image_left, conf=0.3)        #Runs the YOLO algorithm. conf is how confident the model has to be to mark the point
        results_right = self.model(cv_image_right, conf=0.3)
        

        self.save_bounding_box(results_left, left_classes, left_pos_x, left_pos_y, left_boxes, cv_image_left, cv_image_left)
        self.save_bounding_box(results_right, right_classes, right_pos_x, right_pos_y, right_boxes, cv_image_right, cv_image_left)

        self.calculate_depth(cv_image_left, left_classes, left_pos_x, left_pos_y, left_boxes, right_classes, right_pos_x, right_pos_y, right_boxes)

    def save_bounding_box(self, results, classes, pos_x, pos_y, boxes, image_to_use, image_to_draw_on):
        for i in range(len(results[0].boxes)):
            box = results[0].boxes[i]
            x1, y1, x2, y2 = box.xyxy[0].tolist()

            classes.append(int(box.cls[0]))
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

        all_left_boxes = []
        for idx in range(len(left_classes)):
            x1 = left_pos_x[0][idx]
            y1 = left_pos_y[0][idx]
            x2 = left_pos_x[1][idx]
            y2 = left_pos_y[1][idx]
            all_left_boxes.append((x1, y1, x2, y2))

        # Oppdater tracker og få mapping fra deteksjonsindeks -> object_id
        assignments = self.tracker.update(all_left_boxes)

        #Zips information to create one iterable list (those 2 liens are generated using ChatGPT)
        left_objects = [(class_number, (left_pos_x[0][j], left_pos_x[1][j]), (left_pos_y[0][j], left_pos_y[1][j]), j)
                        for j, class_number in enumerate(left_classes)]
        right_objects = [(class_number, (right_pos_x[0][j], right_pos_x[1][j]), (right_pos_y[0][j], right_pos_y[1][j]), j)
                        for j, class_number in enumerate(right_classes)]
        left_objects.sort(key=lambda obj: (obj[2]))
        right_objects.sort(key=lambda obj: (obj[2]))
        left_sorted = [[], [], [], [], []]
        right_sorted = [[], [], [], [], []]

        for obj in left_objects:
            class_number = obj[0]
            left_sorted[class_number].append(obj)
        for obj in right_objects:
             class_number = obj[0]
             right_sorted[class_number].append(obj)
        depth = 0
        for j in range(len(left_sorted)):
            for i in range(min(len(left_sorted[j]), len(right_sorted[j]))):
                left_class, left_x, left_y, left_idx = left_sorted[j][i]
                right_class, right_x, right_y, right_idx = right_sorted[j][i]

                if left_boxes[left_idx].shape[0] < 200 or right_boxes[right_idx].shape[0] < 200:
                    #Uses abs to avoid negative depth. Not correct in theory, but is best solution in practice
                    depth = (self.baseline * self.focal_length) / abs(left_x[0] - right_x[0])
                else:
                    depth = self.orb_calculate_depth(left_boxes[left_idx], right_boxes[right_idx], left_x[0], right_x[0])
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
                    meter_x = ((left_x[0] - self.cx) * depth / self.focal_length + (right_x[0] - self.cx) * depth / self.focal_length)*0.5
                    meter_y = ((left_y[0] - self.cy) * depth / self.focal_length + (right_y[0] - self.cy) * depth / self.focal_length)*0.5
                    self.publish_object_position((left_x[0] + left_x[1]) // 2, (right_x[0] + right_x[1]) // 2, (left_y[0] + left_y[1]) // 2, (right_y[0] + right_y[1]) // 2, depth, left_class)
                    object_id = assignments.get(left_idx)
                    if object_id is not None:
                        timestamp = self.get_clock().now().nanoseconds / 1e9
                        self.csv_logger.writerow([timestamp, object_id, meter_x, meter_y, depth, left_class])
                    if left_class == 0:     #This isn't tested, but should work. If not working, first test should be sending in the entire image, and not just the bounding box.
                        self.publish_aruco_ids(left_boxes[left_idx], right_boxes[right_idx])

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

        keypoints_left, descriptors_left = self.orb.detectAndCompute(cv2.cvtColor(left_box, cv2.COLOR_BGR2GRAY), None)       #Sets the colour to grey, and does orb detection on the images
        keypoints_right, descriptors_right = self.orb.detectAndCompute(cv2.cvtColor(right_box, cv2.COLOR_BGR2GRAY), None)

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