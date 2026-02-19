#This file is similare to the program in main.py, and is created for debugging the PID regulator. Also because I like sexy graphs
import matplotlib.pyplot as plt
import numpy as np

file_path = 'data_files/error_data.csv'
data = np.genfromtxt(file_path, delimiter=',', skip_header=1)

timestamp = data[2:, 0]

error_x = data[2:, 1]
error_y = data[2:, 2]
error_z = data[2:, 3]

error_ori_x = data[2:, 4]
error_ori_y = data[2:, 5]
error_ori_z = data[2:, 6]

startTime = timestamp[0]
for i in range(len(timestamp)):
    timestamp[i] = timestamp[i] - startTime

fig, axes = plt.subplots(3, 2, figsize=(12, 10))
fig.suptitle("Error Plots", fontsize=16)

axes[0, 0].plot(timestamp, error_x)
axes[0, 0].set_title("Position Error X")
axes[0, 0].set_ylabel("Error (m)")

axes[1, 0].plot(timestamp, error_y)
axes[1, 0].set_title("Position Error Y")
axes[1, 0].set_ylabel("Error (m)")

axes[2, 0].plot(timestamp, error_z)
axes[2, 0].set_title("Position Error Z")
axes[2, 0].set_xlabel("Time (s)")
axes[2, 0].set_ylabel("Error (m)")

axes[0, 1].plot(timestamp, error_ori_x)
axes[0, 1].set_title("Roll Error")
axes[0, 1].set_ylabel("Error (quat)")

axes[1, 1].plot(timestamp, error_ori_y)
axes[1, 1].set_title("Pitch Error")
axes[1, 1].set_ylabel("Error (quat)")

axes[2, 1].plot(timestamp, error_ori_z)
axes[2, 1].set_title("Yaw Error")
axes[2, 1].set_xlabel("Time (s)")
axes[2, 1].set_ylabel("Error (quat)")

plt.tight_layout()
plt.show()