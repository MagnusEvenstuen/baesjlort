#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import socket
import struct
import json
import time 

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from mavros_msgs.msg import GPSINPUT

from tf_transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np

def quat_to_rot_matrix(qw, qx, qy, qz):
    """
    Converts a quaternion to a 3x3 rotation matrix.
    Quaternion should be (qw, qx, qy, qz), scalar-first.
    """
    # Precompute repeated terms
    qx2 = qx * qx
    qy2 = qy * qy
    qz2 = qz * qz
    qwqx = qw * qx
    qwqy = qw * qy
    qwqz = qw * qz
    qxqy = qx * qy
    qxqz = qx * qz
    qyqz = qy * qz

    R = np.array([
        [1 - 2*(qy2 + qz2),     2*(qxqy - qwqz),     2*(qxqz + qwqy)],
        [2*(qxqy + qwqz),       1 - 2*(qx2 + qz2),   2*(qyqz - qwqx)],
        [2*(qxqz - qwqy),       2*(qyqz + qwqx),     1 - 2*(qx2 + qy2)]
    ])
    return R



class Patch(Node):
    def __init__(self, node_name, namespace):
        super().__init__(node_name, namespace=namespace)

        self.namespace = self.get_namespace()[1:]

        # Subscribers
        self.create_subscription(Imu, "/gbr/imu_center_perfect", self._imu_callback, 1),
        # self.create_subscription(NavSatFix, "/mavros/global_position/raw/fix", self._gps_callback, 1),
        self.create_subscription(Odometry, "/gbr/odometry_perfect", self._odom_callback, 1),

        # Publishers
        self.pub_pwm = self.create_publisher(Float64MultiArray, "/gbr/thrusters", 1)
        self.pub_fake_gps = self.create_publisher(GPSINPUT, "/mavros/gps_input/gps_input", 1)

        # Publish everything
        self.timer = self.create_timer(1/50, self.looper)

        PORT = 9002

        self.sock_sitl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_sitl.bind(('', PORT))
        self.sock_sitl.settimeout(0.1)

        self.imu = None
        # self.gps = None
        self.odom = None

        self.gps_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # IPV4, UDP
        self.gps_addr = ("127.0.0.1", 25100)

    def _imu_callback(self, msg):
        self.imu = msg

    def _odom_callback(self, msg):
        self.odom = msg

    def looper(self):
        if self.imu is None or self.odom is None:
            self.get_logger().info("Wating for callbacks", once=False)
            time.sleep(.5)
            return
        
        self.get_logger().info("Callbacks received", once=True)
        
        try:
            data, address = self.sock_sitl.recvfrom(100)
        except Exception as ex:
            self.get_logger().info("Socket receive failed, is SITL running?", once=False)
            time.sleep(1)
            return 
    
        parse_format = 'HHI16H'
        magic = 18458

        if len(data) != struct.calcsize(parse_format):
            print("got packet of len %u, expected %u" % (len(data), struct.calcsize(parse_format)))
            return 
        
        decoded = struct.unpack(parse_format,data)

        if magic != decoded[0]:
            print("Incorrect protocol magic %u should be %u" % (decoded[0], magic))
            return 

        print("PWMs", decoded)
        print()

        frame_rate_hz = decoded[1]
        frame_count = decoded[2]
        pwm = decoded[3:]

        pwm_thrusters = pwm[0:8]
        pwm_setpoint = [(x-1500)/400 for x in pwm_thrusters]

        # print(pwm_setpoint)

        # print([pwm[2], pwm[0]])
        # print("{:.2f} {:.2f}".format(pwm_setpoint[0], pwm_setpoint[1]))

        # print(pwm_setpoint)
        msg_pwm = Float64MultiArray(data=pwm_setpoint)

        # Publish pwm message
        self.pub_pwm.publish(msg_pwm)

        # Set mesasges
        accel = (self.imu.linear_acceleration.x, self.imu.linear_acceleration.y, self.imu.linear_acceleration.z)
        gyro = (self.imu.angular_velocity.x, self.imu.angular_velocity.y, self.imu.angular_velocity.z)

        print("Acceleration:", accel)
        print("Gyroscope:", gyro)
        
        pose_position = (
            self.odom.pose.pose.position.x,
            self.odom.pose.pose.position.y,
            self.odom.pose.pose.position.z
        )

        pose_attitude = euler_from_quaternion([
            self.odom.pose.pose.orientation.x,
            self.odom.pose.pose.orientation.y,
            self.odom.pose.pose.orientation.z,
            self.odom.pose.pose.orientation.w
        ])

        # pose_attitude = (
        #     self.odom.pose.pose.orientation.x,
        #     self.odom.pose.pose.orientation.y,
        #     self.odom.pose.pose.orientation.z,
        #     self.odom.pose.pose.orientation.w
        # )
        
        # pose_attitude = [pose_attitude[3], pose_attitude[0], pose_attitude[1], pose_attitude[2]]
        pose_attitude = [pose_attitude[0], pose_attitude[1], pose_attitude[2]]
        # print("Yaw:", np.rad2deg(pose_attitude[2]))
        
        twist_linear = np.array([
            self.odom.twist.twist.linear.x,
            self.odom.twist.twist.linear.y,
            self.odom.twist.twist.linear.z,
            # self.odom.twist.twist.angular.x,
            # self.odom.twist.twist.angular.y,
            # -self.odom.twist.twist.angular.z,
        ])

        R = quat_to_rot_matrix(self.odom.pose.pose.orientation.w, self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z)
        # twist_linear = R@twist_linear
        twist_linear = [twist_linear[0], twist_linear[1], twist_linear[2]]
        
        c_time = self.get_clock().now().to_msg()
        c_time = c_time.sec + c_time.nanosec/1e9

        # build JSON format
        IMU_fmt = {
            "gyro" : gyro,
            "accel_body" : accel
        }
        JSON_fmt = {
            "timestamp" : c_time,
            "imu" : IMU_fmt,
            "position" : pose_position,
            "attitude" : pose_attitude,
            "velocity" : twist_linear,                          
        }
        JSON_string = "\n" + json.dumps(JSON_fmt,separators=(',', ':')) + "\n"

        print("Position:", pose_position)
        print("Orientation:", pose_attitude)
        print("Velocity:", twist_linear)
        print()
        # Send to AP
        self.sock_sitl.sendto(bytes(JSON_string,"ascii"), address)

        fake_gps = GPSINPUT()

        fake_gps.header.stamp = self.get_clock().now().to_msg()
        fake_gps.gps_id = 0
        fake_gps.ignore_flags = 8
        fake_gps.time_week_ms = 0
        fake_gps.time_week = 0
        fake_gps.fix_type = 3
        fake_gps.lat = int((56.026930+pose_position[0]/111320.0)*1e7)
        fake_gps.lon = int((-3.385670+pose_position[1]/111320.0)*1e7)
        # fake_gps.lat = int(56.026930*1e7)
        # fake_gps.lon = int(-3.385670*1e7)
        fake_gps.alt = pose_position[2]
        fake_gps.hdop = 1.0
        fake_gps.vdop = 1.0
        fake_gps.vn = 0.0
        fake_gps.ve = 0.0
        fake_gps.vd = 0.0
        fake_gps.speed_accuracy = 0.0
        fake_gps.horiz_accuracy = 0.0
        fake_gps.vert_accuracy = 0.0
        fake_gps.satellites_visible = 7

        self.pub_fake_gps.publish(fake_gps)


def main(args=None):
    rclpy.init(args=args)

    # patch = Patch(node_name="ardusim_patch")
    patch = Patch(node_name="middleman", namespace='gbr')
    
    rclpy.spin(patch)

    # Destroy the node explicitly, otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    patch.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
