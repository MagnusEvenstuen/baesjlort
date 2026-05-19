import matplotlib.pyplot as plt
import numpy as np
from scipy import signal

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

thruster_1 = data[:, 29]
thruster_2 = data[:, 30]
thruster_3 = data[:, 31]
thruster_4 = data[:, 32]
thruster_5 = data[:, 33]
thruster_6 = data[:, 34]
thruster_7 = data[:, 35]
thruster_8 = data[:, 36]

SLAM_counter = data[:, 37]

print("Average thruster 1", np.average(np.abs(thruster_1)))
print("Average thruster 2", np.average(np.abs(thruster_2)))
print("Average thruster 3", np.average(np.abs(thruster_3)))
print("Average thruster 4", np.average(np.abs(thruster_4)))
print("Average thruster 5", np.average(np.abs(thruster_5)))
print("Average thruster 6", np.average(np.abs(thruster_6)))
print("Average thruster 7", np.average(np.abs(thruster_7)))
print("Average thruster 8", np.average(np.abs(thruster_8)))

print("Variance thruster 1", np.var(thruster_1))
print("Variance thruster 2", np.var(thruster_2))
print("Variance thruster 3", np.var(thruster_3))
print("Variance thruster 4", np.var(thruster_4))
print("Variance thruster 5", np.var(thruster_5))
print("Variance thruster 6", np.var(thruster_6))
print("Variance thruster 7", np.var(thruster_7))
print("Variance thruster 8", np.var(thruster_8))

depth = data[:, 19]

#Changes start value of true positions to be same as on measured (Only check position compared to start)
#1 is used instead of 0 as index, because sometimes the first value is from before data is rescieved from the odometry sensor
correct_pos_x_start = data[:, 16][1]
correct_pos_y_start = data[:, 17][1]
correct_pos_z_start = data[:, 18][1]

#This is used for finding the R matrix in the kalman filter. Placed here and not in SYSID due to SYSID not having implemented stationary data yet. Might be done later
print("Variance ax:", acc_x.var(), "ay:", acc_y.var(), "az:", acc_z.var())
print("Variance gx:", gyro_x.var(), "gy:", gyro_y.var(), "gz:", gyro_z.var())
print("Average ax:", acc_x.mean(), "ay:", acc_y.mean(), "az:", acc_z.mean())
print("Average gx:", gyro_x.mean(), "gy:", gyro_y.mean(), "gz:", gyro_z.mean())

for i in range(len(correct_pos_x)):
    correct_pos_x[i] -= correct_pos_x_start
    correct_pos_y[i] -= correct_pos_y_start
    correct_pos_z[i] -= correct_pos_z_start

plt.figure()
plt.subplot(4,1,1)
plt.title("Acceleration")
plt.plot(timestamp, acc_x, label="acceleration_x", color="red")
plt.plot(timestamp, acc_y, label="acceleration_y", color="blue")
plt.plot(timestamp, acc_z, label="acceleration_z", color="green")
#plt.plot(timestamp, correct_acc_x, label="correct_acceleration_x", linestyle="--", color="red")
#plt.plot(timestamp, correct_acc_y, label="correct_acceleration_y", linestyle="--", color="blue")
#plt.plot(timestamp, correct_acc_z, label="correct_acceleration_z", linestyle="--", color="green")
plt.xlabel("Time(s)")
plt.ylabel("Acceleration (m/s^2)")
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

plt.subplot(4,1,2)
plt.title("Velocity")
plt.plot(timestamp, vel_x, label="velocity_x", color="red")
plt.plot(timestamp, vel_y, label="velocity_y", color="blue")
plt.plot(timestamp, vel_z, label="velocity_z", color="green")
#plt.plot(timestamp, correct_vel_x, label="correct_velocity_x", linestyle="--", color="red")
#plt.plot(timestamp, correct_vel_y, label="correct_velocity_y", linestyle="--", color="blue")
#plt.plot(timestamp, correct_vel_z, label="correct_velocity_z", linestyle="--", color="green")
plt.xlabel("Time(s)")
plt.ylabel("Speed (m/s)")
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

plt.subplot(4,1,3)
plt.title("Position")
plt.plot(timestamp, pos_x, label="position_x", color="red")
plt.plot(timestamp, pos_y, label="position_y", color="blue")
plt.plot(timestamp, pos_z, label="position_z", color="green")
#plt.plot(timestamp, correct_pos_x, label="correct_position_x", linestyle="--", color="red")
#plt.plot(timestamp, correct_pos_y, label="correct_position_y", linestyle="--", color="blue")
#plt.plot(timestamp, correct_pos_z, label="correct_position_z", linestyle="--", color="green")
#plt.plot(timestamp, depth, label="depth")
plt.xlabel("Time(s)")
plt.ylabel("Position (m)")
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

plt.subplot(4,1,4)
plt.title("Orientation")
plt.plot(timestamp, orientation_x, label="orientation_x", color="red")
plt.plot(timestamp, orientation_y, label="orientation_y", color="blue")
plt.plot(timestamp, orientation_z, label="orientation_z", color="green")
#plt.plot(timestamp, correct_orientation_x, label="correct_orientation_x", linestyle="--", color="red")
#plt.plot(timestamp, correct_orientation_y, label="correct_orientation_y", linestyle="--", color="blue")
#plt.plot(timestamp, correct_orientation_z, label="correct_orientation_z", linestyle="--", color="green")
plt.xlabel("Time(s)")
plt.ylabel("Orientation")
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

fig, axs = plt.subplots(4, 2, figsize=(12, 16))

axs[0, 0].plot(timestamp, thruster_1, label="thruster_1")
axs[0, 1].plot(timestamp, thruster_2, label="thruster_2")
axs[1, 0].plot(timestamp, thruster_3, label="thruster_3")
axs[1, 1].plot(timestamp, thruster_4, label="thruster_4")
axs[2, 0].plot(timestamp, thruster_5, label="thruster_5")
axs[2, 1].plot(timestamp, thruster_6, label="thruster_6")
axs[3, 0].plot(timestamp, thruster_7, label="thruster_7")
axs[3, 1].plot(timestamp, thruster_8, label="thruster_8")

for i in range(4):
    for j in range(2):
        axs[i, j].set_title(f"thruster_{i*2 + j + 1}")
        axs[i, j].set_xlabel("Time(s)")
        axs[i, j].set_ylabel("Command")
        axs[i, j].legend()

plt.tight_layout()
plt.show()

orientation = np.array([1.0, 0.0, 0.0, 0.0])  # w, x, y, z
velocity = np.zeros(3)
position = np.zeros(3)

# Arrays for å lagre den integrerte posisjonen (samme fortegn som CSV-en)
integrated_x = np.zeros(len(data))
integrated_y = np.zeros(len(data))
integrated_z = np.zeros(len(data))

for i in range(1, len(data)):  # hopp over indeks 0 fordi dt=0
    # Tidssteg (samme dt som i C++)
    dt = timestamp[i] - timestamp[i-1]

    # Gyro (vinkelhastighet i kroppsramme) – fra CSV (filtrert av Kalman)
    gx, gy, gz = gyro_x[i], gyro_y[i], gyro_z[i]

    # Oppdater orientering: delta_q = (1, gx*dt/2, gy*dt/2, gz*dt/2)
    angle = 0.5 * dt * np.array([gx, gy, gz])
    delta_q = np.array([1.0, angle[0], angle[1], angle[2]])
    # Quaternion-multiplikasjon (C++: orientation_ = orientation_ * delta_orientation)
    w1, x1, y1, z1 = orientation
    dw, dx, dy, dz = delta_q
    orientation = np.array([
        w1*dw - x1*dx - y1*dy - z1*dz,
        w1*dx + x1*dw + y1*dz - z1*dy,
        w1*dy - x1*dz + y1*dw + z1*dx,
        w1*dz + x1*dy - y1*dx + z1*dw
    ])
    orientation /= np.linalg.norm(orientation)  # normaliser

    # Akselerasjon (kroppsramme) – fra CSV (filtrert av Kalman, alt gravitasjonskompensert)
    ax, ay, az = acc_x[i], acc_y[i], acc_z[i]
    acc_body = np.array([ax, ay, az])

    # *** FIKSE: Roter akselerasjonen fra kroppsramme til verdensramme ***
    qw, qx, qy, qz = orientation
    # Rotasjonsmatrise fra kvaternion
    R = np.array([
        [1 - 2*(qy**2 + qz**2),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),         1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),         2*(qy*qz + qx*qw),     1 - 2*(qx**2 + qy**2)]
    ])
    acc_world = R @ acc_body

    # Hastighet: v += a * dt (nå med verdensakselerasjon)
    velocity += acc_world * dt

    # Posisjon: p -= v * dt (minus-fortegn som i C++)
    position -= velocity * dt

    # Lagre i CSV-formatets fortegn (slik de logges)
    # C++ logger: -pos.x(), pos.y(), -pos.z()
    integrated_x[i] = -position[0]
    integrated_y[i] =  position[1]
    integrated_z[i] = -position[2]

slam_counter = 0
for i in range(len(SLAM_counter)):
    if SLAM_counter[i] == 1:
        slam_counter += 1

print("Final position world: X:", pos_x[-1], "Y:", pos_y[-1], "Z:", pos_z[-1])
print("Final position integrated IMU:", integrated_x[-1], "Y:", integrated_y[-1], "Z:", integrated_z[-1])
print("Error final position world:", np.sqrt(pos_x[-1]**2+pos_y[-1]**2+pos_z[-1]**2))
print("Error final position IMU world:", np.sqrt(integrated_x[-1]**2+integrated_y[-1]**2+integrated_z[-1]**2))
print("SLAM tracking percent:", slam_counter/2280)
print("Test time:", timestamp[-1] - timestamp[0])