import matplotlib.pyplot as pp
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles, QoSProfile
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header
from rclpy.time import Time
import pandas as pd

class Plotter(Node):
    def __init__(self):
        super().__init__("plotter")

        self.img_sub = self.create_subscription(Image, "/gbr/cam_left/image_raw", self.image_callback, QoSPresetProfiles.BEST_AVAILABLE.value)

        self.img_list = []
        self.timer = self.create_timer(5, self.timer_callback)
        self.c = 0

    def image_callback(self, img):
        self.timer.reset()
        self.img_list.append(Time().from_msg(img.header.stamp).nanoseconds)
        self.c += 1

    def timer_callback(self):
        df = pd.DataFrame(self.img_list, columns=["timestamp"])
        df.to_csv("gpu.csv")
        print(f"Killing myself, {self.c}")
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    plotter = Plotter()
    rclpy.spin(plotter)

if __name__ == '__main__':
    main()

