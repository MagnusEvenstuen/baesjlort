#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.left_cam = self.create_publisher(Image, '/gbr/cam_left/image_color', 30)
        self.right_cam = self.create_publisher(Image, '/gbr/cam_right/image_color', 30)
        self.bridge = CvBridge()
        
        self.left_path = "/home/gud/Skole/baesjlort/scripts/plotting_program/videos/left_slow3.mp4"
        self.capture_left = cv2.VideoCapture(self.left_path)

        self.right_path = "/home/gud/Skole/baesjlort/scripts/plotting_program/videos/right_slow3.mp4"
        self.capture_right = cv2.VideoCapture(self.right_path)

        self.fps = 30
        self.frame_delay = 1.0 / self.fps
        
        self.init_rectification_maps()
        
        self.timer = self.create_timer(self.frame_delay, self.publish_frame)
    
    def init_rectification_maps(self):
        self.image_size = (640, 480)
        
        K_left = np.array([[449.56321,   0.     , 313.40058],
                           [  0.     , 450.31696, 242.32592],
                           [  0.     ,   0.     ,   1.     ]], dtype=np.float64)
        D_left = np.array([-0.022368, 0.028507, -0.002649, -0.003014, 0.000000], dtype=np.float64)
        R_left = np.array([[ 0.97734046, -0.00799868,  0.21152222],
                           [ 0.00862238,  0.99996077, -0.00202643],
                           [-0.21149771,  0.00380433,  0.97737109]], dtype=np.float64)
        P_left = np.array([[538.78453,   0.     , 161.95527,   0.     ],
                           [  0.     , 538.78453, 230.65832,   0.     ],
                           [  0.     ,   0.     ,   1.     ,   0.     ]], dtype=np.float64)
        
        # Right camera parameters
        K_right = np.array([[462.2107 ,   0.     , 299.81391],
                            [  0.     , 460.94032, 223.64644],
                            [  0.     ,   0.     ,   1.     ]], dtype=np.float64)
        D_right = np.array([-0.006540, 0.020252, -0.002126, -0.007280, 0.000000], dtype=np.float64)
        R_right = np.array([[ 0.98311934, -0.0079423 ,  0.18279299],
                            [ 0.00740403,  0.99996601,  0.00362695],
                            [-0.18281559, -0.00221232,  0.98314473]], dtype=np.float64)
        P_right = np.array([[538.78453,   0.     , 161.95527,  23.60963],
                            [  0.     , 538.78453, 230.65832,   0.     ],
                            [  0.     ,   0.     ,   1.     ,   0.     ]], dtype=np.float64)
        
        # Compute rectification maps
        self.map_left_x, self.map_left_y = cv2.initUndistortRectifyMap(
            K_left, D_left, R_left, P_left, self.image_size, cv2.CV_32FC1)
        self.map_right_x, self.map_right_y = cv2.initUndistortRectifyMap(
            K_right, D_right, R_right, P_right, self.image_size, cv2.CV_32FC1)
        
        self.get_logger().info("Rectification maps created.")
    
    def publish_frame(self):
        ret_left, frame_left = self.capture_left.read()
        ret_right, frame_right = self.capture_right.read()
        if not ret_left or not ret_right:
            self.get_logger().info('Video ferdig, avslutter...')
            rclpy.shutdown()
            return

        rect_left = cv2.remap(frame_left, self.map_left_x, self.map_left_y, cv2.INTER_LINEAR)
        rect_right = cv2.remap(frame_right, self.map_right_x, self.map_right_y, cv2.INTER_LINEAR)

        # ---------- VISUALISERING AV ORB ----------
        left_display = rect_left.copy()
        right_display = rect_right.copy()

        orb = cv2.ORB_create(
            nfeatures=1500,
            scaleFactor=1.2,
            nlevels=8,
            edgeThreshold=31,
            patchSize=31,
            fastThreshold=3
        )

        kp_left = orb.detect(rect_left, None)
        kp_right = orb.detect(rect_right, None)

        keypoints_left, descriptors_left = orb.detectAndCompute(
        cv2.cvtColor(rect_left, cv2.COLOR_BGR2GRAY), None)
        keypoints_right, descriptors_right = orb.detectAndCompute(
        cv2.cvtColor(rect_right, cv2.COLOR_BGR2GRAY), None)

        self.get_logger().info(f"Left descriptors: {len(descriptors_left)}, Right descriptors: {len(descriptors_right)}")

        for kp in kp_left:
            x, y = int(kp.pt[0]), int(kp.pt[1])
            cv2.circle(left_display, (x, y), 1, (0,255,0), -1)
        for kp in kp_right:
            x, y = int(kp.pt[0]), int(kp.pt[1])
            cv2.circle(right_display, (x, y), 1, (0,255,0), -1)

        vis = np.hstack([left_display, right_display])

        cv2.imshow("ORB Features & Rectification", vis)
        cv2.waitKey(1)

        ros_image_left = self.bridge.cv2_to_imgmsg(rect_left, encoding='bgr8')
        ros_image_right = self.bridge.cv2_to_imgmsg(rect_right, encoding='bgr8')
        
        now = self.get_clock().now().to_msg()
        ros_image_left.header.stamp = now
        ros_image_left.header.frame_id = 'camera_frame'
        ros_image_right.header.stamp = now
        ros_image_right.header.frame_id = 'camera_frame'
        
        self.left_cam.publish(ros_image_left)
        self.right_cam.publish(ros_image_right)
    
    def destroy_node(self):
        self.capture_left.release()
        self.capture_right.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()