#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import struct
from std_msgs.msg import Header

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.left_cam = self.create_publisher(Image, '/gbr/cam_left/image_color', 30)
        self.right_cam = self.create_publisher(Image, '/gbr/cam_right/image_color', 30)
        self.pose_pub = self.create_publisher(PoseStamped, '/slam/pose', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/slam/pointcloud', 10)
        self.bridge = CvBridge()
        
        self.left_path = "/home/gud/Skole/baesjlort/scripts/plotting_program/videos/right_slow.mp4"
        self.capture_left = cv2.VideoCapture(self.left_path)

        self.right_path = "/home/gud/Skole/baesjlort/scripts/plotting_program/videos/left_slow.mp4"
        self.capture_right = cv2.VideoCapture(self.right_path)

        self.fps = 30
        self.frame_delay = 1.0 / self.fps
        
        self.init_rectification_maps()
        
        self.orb = cv2.ORB_create(
            nfeatures=1500,
            scaleFactor=1.2,
            nlevels=8,
            edgeThreshold=31,
            patchSize=31,
            fastThreshold=3
        )
        
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        
        self.prev_left_gray = None
        self.prev_kp = None
        self.prev_des = None
        self.prev_pts_left = []
        self.prev_pts_3d = []
        self.prev_pose = np.eye(4)
        
        self.global_landmarks = []
        
        # Reset‑mekanisme
        self.low_match_counter = 0
        self.reset_threshold = 5          # antall påfølgende frames med for få matcher før reset
        
        # Kameraintrinsics
        self.fx = 538.78453
        self.fy = 538.78453
        self.cx = 161.95527
        self.cy = 230.65832
        self.baseline = 23.60963 / self.fx
        
        self.camera_matrix = np.array([[self.fx, 0, self.cx],
                                       [0, self.fy, self.cy],
                                       [0, 0, 1]], dtype=np.float32)
        
        self.timer = self.create_timer(self.frame_delay, self.publish_frame)
        self.get_logger().info("Stereo VO node med automatisk reset ved lave matcher.")
    
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
        
        self.map_left_x, self.map_left_y = cv2.initUndistortRectifyMap(
            K_left, D_left, R_left, P_left, self.image_size, cv2.CV_32FC1)
        self.map_right_x, self.map_right_y = cv2.initUndistortRectifyMap(
            K_right, D_right, R_right, P_right, self.image_size, cv2.CV_32FC1)
        
        self.get_logger().info("Rectification maps created.")
    
    def triangulate_stereo(self, kp_left, des_left, kp_right, des_right):
        if des_left is None or des_right is None or len(kp_left) == 0 or len(kp_right) == 0:
            return [], [], []
        
        raw_matches = self.bf.match(des_left, des_right)
        if len(raw_matches) == 0:
            return [], [], []
        
        raw_matches = sorted(raw_matches, key=lambda x: x.distance)[:100]
        
        pts_left = []
        pts_right = []
        points_3d = []
        good_matches = []
        
        for m in raw_matches:
            pt_left = kp_left[m.queryIdx].pt
            pt_right = kp_right[m.trainIdx].pt
            
            disparity = pt_left[0] - pt_right[0]
            if disparity <= 0.1:
                continue
            
            depth = self.fx * self.baseline / disparity
            if depth > 200.0 or depth < 0.1:
                continue
            
            x = (pt_left[0] - self.cx) * depth / self.fx
            y = (pt_left[1] - self.cy) * depth / self.fy
            point = np.array([x, y, depth])
            
            good_matches.append(m)
            pts_left.append(pt_left)
            pts_right.append(pt_right)
            points_3d.append(point)
        
        self.get_logger().info(f"Stereo matches etter filtrering: {len(pts_left)}")
        return pts_left, points_3d, good_matches
    
    def estimate_motion(self, pts_3d_prev, pts_2d_curr):
        if len(pts_3d_prev) < 4 or len(pts_2d_curr) < 4:
            return None, None
        
        pts_3d = np.array(pts_3d_prev, dtype=np.float32)
        pts_2d = np.array(pts_2d_curr, dtype=np.float32).reshape(-1, 2)
        
        _, rvec, tvec, inliers = cv2.solvePnPRansac(
            pts_3d, pts_2d, self.camera_matrix, None,
            iterationsCount=100, reprojectionError=8.0, confidence=0.9,
            flags=cv2.SOLVEPNP_ITERATIVE
        )
        
        if inliers is None or len(inliers) < 4:
            return None, None
        
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = tvec.flatten()
        
        # Forkast usannsynlig store bevegelser (> 10 meter på én frame)
        trans_norm = np.linalg.norm(T[:3, 3])
        if trans_norm > 10.0:
            self.get_logger().warn(f"Ekstrem bevegelse ({trans_norm:.1f} m) – forkaster.")
            return None, None
        
        return T, inliers
    
    def match_frames(self, des_prev, des_curr, kp_prev, kp_curr, pts_3d_prev, pts_left_prev):
        if des_prev is None or des_curr is None:
            return [], [], []
        
        matches = self.bf.match(des_prev, des_curr)
        if len(matches) == 0:
            return [], [], []
        
        matches = sorted(matches, key=lambda x: x.distance)[:200]
        
        pts_prev = []
        pts_curr = []
        pts_3d_matched = []
        
        prev_2d_to_3d = {}
        for pt, pt3d in zip(pts_left_prev, pts_3d_prev):
            key = (int(pt[0]), int(pt[1]))
            prev_2d_to_3d[key] = pt3d
        
        for m in matches:
            pt_prev = kp_prev[m.queryIdx].pt
            pt_curr = kp_curr[m.trainIdx].pt
            key = (int(pt_prev[0]), int(pt_prev[1]))
            if key in prev_2d_to_3d:
                pts_prev.append(pt_prev)
                pts_curr.append(pt_curr)
                pts_3d_matched.append(prev_2d_to_3d[key])
        
        self.get_logger().info(f"Matchet {len(pts_curr)} features med 3D-punkter.")
        return pts_prev, pts_curr, pts_3d_matched
    
    def update_global_pose(self, delta_T):
        self.prev_pose = self.prev_pose @ delta_T
        return self.prev_pose
    
    def transform_points_to_world(self, points_cam, pose):
        points_world = []
        for p in points_cam:
            p_hom = np.array([p[0], p[1], p[2], 1.0])
            p_w = pose @ p_hom
            points_world.append(p_w[:3])
        return points_world
    
    def reset_slam(self):
        """Nullstiller hele SLAM‑tilstanden."""
        self.get_logger().warn("RESETTER SLAM pga. for mange frames med lave matcher.")
        self.prev_left_gray = None
        self.prev_kp = None
        self.prev_des = None
        self.prev_pts_left = []
        self.prev_pts_3d = []
        self.prev_pose = np.eye(4)
        self.global_landmarks = []
        self.low_match_counter = 0
    
    def publish_pose(self, pose_matrix, timestamp):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = "world"
        
        pose_msg.pose.position.x = pose_matrix[0, 3]
        pose_msg.pose.position.y = pose_matrix[1, 3]
        pose_msg.pose.position.z = pose_matrix[2, 3]
        
        R = pose_matrix[:3, :3]
        qw = math.sqrt(1.0 + R[0,0] + R[1,1] + R[2,2]) / 2.0
        if qw > 1e-6:
            qx = (R[2,1] - R[1,2]) / (4.0 * qw)
            qy = (R[0,2] - R[2,0]) / (4.0 * qw)
            qz = (R[1,0] - R[0,1]) / (4.0 * qw)
        else:
            qx = qy = qz = 0.0
            qw = 1.0
        
        pose_msg.pose.orientation.w = qw
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        
        self.pose_pub.publish(pose_msg)
    
    def publish_pointcloud(self, points, timestamp):
        if len(points) == 0:
            return
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        header = Header(stamp=timestamp, frame_id="world")
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.fields = fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.is_dense = True
        
        data = []
        for p in points:
            data.append(struct.pack('fff', p[0], p[1], p[2]))
        cloud_msg.data = b''.join(data)
        
        self.cloud_pub.publish(cloud_msg)
    
    def publish_frame(self):
        ret_left, frame_left = self.capture_left.read()
        ret_right, frame_right = self.capture_right.read()
        if not ret_left or not ret_right:
            self.get_logger().info('Video ferdig, avslutter...')
            rclpy.shutdown()
            return

        rect_left = cv2.remap(frame_left, self.map_left_x, self.map_left_y, cv2.INTER_LINEAR)
        rect_right = cv2.remap(frame_right, self.map_right_x, self.map_right_y, cv2.INTER_LINEAR)

        gray_left = cv2.cvtColor(rect_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(rect_right, cv2.COLOR_BGR2GRAY)
        
        kp_left, des_left = self.orb.detectAndCompute(gray_left, None)
        kp_right, des_right = self.orb.detectAndCompute(gray_right, None)
        
        self.get_logger().info(f"Keypoints left: {len(kp_left)}, right: {len(kp_right)}")
        
        pts_left_curr, pts_3d_curr, stereo_matches = self.triangulate_stereo(kp_left, des_left, kp_right, des_right)
        self.get_logger().info(f"Stereo matches (brukbare): {len(pts_3d_curr)}")
        
        # Visning av stereo‑matcher
        if len(stereo_matches) > 0:
            match_img = cv2.drawMatches(rect_left, kp_left, rect_right, kp_right, 
                                        stereo_matches[:50], None, 
                                        flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
            cv2.imshow("Stereo Matches", match_img)
        else:
            vis = np.hstack([rect_left, rect_right])
            cv2.imshow("Stereo Matches", vis)
        cv2.waitKey(1)
        
        if self.prev_left_gray is None:
            self.prev_left_gray = gray_left
            self.prev_kp = kp_left
            self.prev_des = des_left
            self.prev_pts_left = pts_left_curr
            self.prev_pts_3d = pts_3d_curr
            self.prev_pose = np.eye(4)
            world_pts = self.transform_points_to_world(pts_3d_curr, self.prev_pose)
            self.global_landmarks.extend(world_pts)
            self.get_logger().info(f"Initialisert med {len(world_pts)} 3D-punkter.")
        else:
            pts_prev, pts_curr, pts_3d_matched = self.match_frames(
                self.prev_des, des_left, self.prev_kp, kp_left,
                self.prev_pts_3d, self.prev_pts_left
            )
            
            if len(pts_curr) >= 4:
                # Nullstill telleren når vi har nok matcher
                self.low_match_counter = 0
                
                delta_T, inliers = self.estimate_motion(pts_3d_matched, pts_curr)
                if delta_T is not None:
                    self.update_global_pose(delta_T)
                    self.get_logger().info(f"Bevegelse: tx={delta_T[0,3]:.3f}, tz={delta_T[2,3]:.3f}")
                    
                    total_dx = self.prev_pose[0, 3]
                    total_dy = self.prev_pose[1, 3]
                    total_dz = self.prev_pose[2, 3]
                    total_dist = math.sqrt(total_dx**2 + total_dy**2 + total_dz**2)
                    self.get_logger().info(f"Total bevegelse: dx={total_dx:.3f}, dy={total_dy:.3f}, dz={total_dz:.3f}, dist={total_dist:.3f} m")
                    
                    world_pts = self.transform_points_to_world(pts_3d_curr, self.prev_pose)
                    self.global_landmarks.extend(world_pts)
                    
                    if len(self.global_landmarks) > 50000:
                        self.global_landmarks = self.global_landmarks[-50000:]
                else:
                    self.get_logger().warn("PnP feilet.")
                    # Feilet estimering teller også som lav match
                    self.low_match_counter += 1
            else:
                self.get_logger().warn(f"For få matches for bevegelse: {len(pts_curr)}")
                self.low_match_counter += 1
                if self.low_match_counter >= self.reset_threshold:
                    self.reset_slam()
                    # Etter reset, bruk nåværende frame som ny initialisering
                    self.prev_left_gray = gray_left
                    self.prev_kp = kp_left
                    self.prev_des = des_left
                    self.prev_pts_left = pts_left_curr
                    self.prev_pts_3d = pts_3d_curr
                    self.prev_pose = np.eye(4)
                    world_pts = self.transform_points_to_world(pts_3d_curr, self.prev_pose)
                    self.global_landmarks = world_pts
                    self.get_logger().info(f"Re‑initialisert med {len(world_pts)} 3D-punkter.")
                    # Publiser pose og punktky med en gang
                    now = self.get_clock().now().to_msg()
                    self.publish_pose(self.prev_pose, now)
                    self.publish_pointcloud(self.global_landmarks, now)
                    return  # hopp over resten av funksjonen for denne framen
            
            # Oppdater forrige frame (hvis ikke reset)
            self.prev_left_gray = gray_left
            self.prev_kp = kp_left
            self.prev_des = des_left
            self.prev_pts_left = pts_left_curr
            self.prev_pts_3d = pts_3d_curr
        
        now = self.get_clock().now().to_msg()
        self.publish_pose(self.prev_pose, now)
        self.publish_pointcloud(self.global_landmarks, now)
        
        # Visualisering av keypoints
        left_display = rect_left.copy()
        right_display = rect_right.copy()
        for kp in kp_left:
            cv2.circle(left_display, (int(kp.pt[0]), int(kp.pt[1])), 1, (0,255,0), -1)
        for kp in kp_right:
            cv2.circle(right_display, (int(kp.pt[0]), int(kp.pt[1])), 1, (0,255,0), -1)
        vis = np.hstack([left_display, right_display])
        cv2.imshow("ORB Features & Rectification", vis)
        cv2.waitKey(1)

        ros_image_left = self.bridge.cv2_to_imgmsg(rect_left, encoding='bgr8')
        ros_image_right = self.bridge.cv2_to_imgmsg(rect_right, encoding='bgr8')
        ros_image_left.header.stamp = now
        ros_image_left.header.frame_id = 'camera_frame'
        ros_image_right.header.stamp = now
        ros_image_right.header.frame_id = 'camera_frame'
        self.left_cam.publish(ros_image_left)
        self.right_cam.publish(ros_image_right)
    
    def destroy_node(self):
        self.capture_left.release()
        self.capture_right.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()