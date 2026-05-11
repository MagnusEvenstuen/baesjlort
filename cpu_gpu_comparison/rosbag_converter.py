from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
import pandas as pd
from rclpy.time import Time

compressed_dict = { "cam_left_stamp": [], "cam_right_stamp": [] }
cpu_dict = { "cam_left_stamp": [], "cam_right_stamp": [] }
gpu_dict = { "cam_left_stamp": [], "cam_right_stamp": [] }

typestore = get_typestore(Stores.ROS2_JAZZY)

with Reader("rosbag2_compressed") as reader:
    for connection, timestamp, rawdata in reader.messages():
        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)

        if connection.topic == "/gbr/cam_left/image_compressed":
            ts = Time(seconds=msg.header.stamp.sec, nanoseconds=msg.header.stamp.nanosec)
            compressed_dict["cam_left_stamp"].append(ts.nanoseconds)
        elif connection.topic == "/gbr/cam_right/image_compressed":
            ts = Time(seconds=msg.header.stamp.sec, nanoseconds=msg.header.stamp.nanosec)
            compressed_dict["cam_right_stamp"].append(ts.nanoseconds)

with Reader("rosbag2_cpu_640x480") as reader:
    for connection, timestamp, rawdata in reader.messages():
        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)

        if connection.topic == "/gbr/cam_left/image_raw":
            ts = Time(seconds=msg.header.stamp.sec, nanoseconds=msg.header.stamp.nanosec)
            cpu_dict["cam_left_stamp"].append(ts.nanoseconds)
        elif connection.topic == "/gbr/cam_right/image_raw":
            ts = Time(seconds=msg.header.stamp.sec, nanoseconds=msg.header.stamp.nanosec)
            cpu_dict["cam_right_stamp"].append(ts.nanoseconds)

with Reader("rosbag2_gpu_640x480") as reader:
    for connection, timestamp, rawdata in reader.messages():
        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)

        if connection.topic == "/gbr/cam_left/image_raw":
            ts = Time(seconds=msg.header.stamp.sec, nanoseconds=msg.header.stamp.nanosec)
            gpu_dict["cam_left_stamp"].append(ts.nanoseconds)
        elif connection.topic == "/gbr/cam_right/image_raw":
            ts = Time(seconds=msg.header.stamp.sec, nanoseconds=msg.header.stamp.nanosec)
            gpu_dict["cam_right_stamp"].append(ts.nanoseconds)

# Allow different length lists
df = pd.DataFrame.from_dict(compressed_dict, orient="index").T
df.to_csv('compressed.csv', index=False)

df = pd.DataFrame.from_dict(cpu_dict, orient="index").T
df.to_csv('cpu.csv', index=False)

df = pd.DataFrame.from_dict(gpu_dict, orient="index").T
df.to_csv('gpu.csv', index=False)
