import matplotlib.pyplot as plt
import numpy as np
import os

file_path = 'data_files/sensor_data.csv'
data = np.genfromtxt(file_path, delimiter=',', skip_header=1)


timestamp = data[:, 0]

acc_x = data[:, 1]
acc_y = data[:, 2]
acc_z = data[:, 3]

vel_x = data[:, 4]
vel_y = data[:, 5]
vel_z = data[:, 6]

pos_x = data[:, 7]
pos_y = data[:, 8]
pos_z = data[:, 9]

correct_acc_x = data[:, 10]
correct_acc_y = data[:, 11]
correct_acc_z = data[:, 12]

correct_vel_x = data[:, 13]
correct_vel_y = data[:, 14]
correct_vel_z = data[:, 15]

correct_pos_x = data[:, 16]
correct_pos_y = data[:, 17]
correct_pos_z = data[:, 18]

orientation_x = data[:, 20]
orientation_y = data[:, 21]
orientation_z = data[:, 22]

correct_orientation_x = data[:, 23]
correct_orientation_y = data[:, 24]
correct_orientation_z = data[:, 25]

gyro_x = data[:, 26]
gyro_y = data[:, 27]
gyro_z = data[:, 28]

depth = data[:, 19]

#Changes start value of true positions to be same as on measured (Only check position compared to start)
#1 is used instead of 0 as index, because sometimes the first value is from before data is rescieved from the odometry sensor
correct_pos_x_start = data[:, 16][1]
correct_pos_y_start = data[:, 17][1]
correct_pos_z_start = data[:, 18][1]

#This is used for finding the R matrix in the kalman filter. Placed here and not in SYSID due to SYSID not having implemented stationary data yet. Might be done later
print("ax:", acc_x.var(), "ay:", acc_y.var(), "az:", acc_z.var())
print("gx:", gyro_x.var(), "gy:", gyro_y.var(), "gz:", gyro_z.var())

for i in range(len(correct_pos_x)):
    correct_pos_x[i] -= correct_pos_x_start
    correct_pos_y[i] -= correct_pos_y_start
    correct_pos_z[i] -= correct_pos_z_start

plt.figure()
plt.subplot(4,1,1)
plt.title("Acceleration")
plt.plot(timestamp, acc_x, label="acceleration_x", color="red")
plt.plot(timestamp, acc_y, label="acceleration_y", color="blue")
#plt.plot(timestamp, acc_z, label="acceleration_z", color="green")
#plt.plot(timestamp, correct_acc_x, label="correct_acceleration_x", linestyle="--", color="red")
#plt.plot(timestamp, correct_acc_y, label="correct_acceleration_y", linestyle="--", color="blue")
#plt.plot(timestamp, correct_acc_z, label="correct_acceleration_z", linestyle="--", color="green")
plt.xlabel("Time")
plt.ylabel("Acceleration (m/s^2)")
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

plt.subplot(4,1,2)
plt.title("Velocity")
plt.plot(timestamp, vel_x, label="velocity_x", color="red")
plt.plot(timestamp, vel_y, label="velocity_y", color="blue")
plt.plot(timestamp, vel_z, label="velocity_z", color="green")
plt.plot(timestamp, correct_vel_x, label="correct_velocity_x", linestyle="--", color="red")
plt.plot(timestamp, correct_vel_y, label="correct_velocity_y", linestyle="--", color="blue")
plt.plot(timestamp, correct_vel_z, label="correct_velocity_z", linestyle="--", color="green")
plt.xlabel("Time")
plt.ylabel("Speed (m/s)")
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

plt.subplot(4,1,3)
plt.title("Position")
plt.plot(timestamp, pos_x, label="position_x", color="red")
plt.plot(timestamp, pos_y, label="position_y", color="blue")
plt.plot(timestamp, pos_z, label="position_z", color="green")
plt.plot(timestamp, correct_pos_x, label="correct_position_x", linestyle="--", color="red")
plt.plot(timestamp, correct_pos_y, label="correct_position_y", linestyle="--", color="blue")
plt.plot(timestamp, correct_pos_z, label="correct_position_z", linestyle="--", color="green")
#plt.plot(timestamp, depth, label="depth")
plt.xlabel("Time")
plt.ylabel("Position (m)")
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

plt.subplot(4,1,4)
plt.title("Orientation")
plt.plot(timestamp, orientation_x, label="orientation_x", color="red")
plt.plot(timestamp, orientation_y, label="orientation_y", color="blue")
plt.plot(timestamp, orientation_z, label="orientation_z", color="green")
plt.plot(timestamp, correct_orientation_x, label="correct_orientation_x", linestyle="--", color="red")
plt.plot(timestamp, correct_orientation_y, label="correct_orientation_y", linestyle="--", color="blue")
plt.plot(timestamp, correct_orientation_z, label="correct_orientation_z", linestyle="--", color="green")
plt.xlabel("Time")
plt.ylabel("Orientation")
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

plt.show()