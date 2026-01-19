#Code here has inspiration from https://github.com/MagnusEvenstuen/kybernetikk_robotarm/blob/master/DataLeser/SysID.py
import pandas as pd
import numpy as np
from sippy_unipi import system_identification as sysid
from sippy_unipi import functionset as fset
import matplotlib.pyplot as plt
import control as ct
import sklearn

file_path_training = 'data_files/training_data.csv'
file_path_validation = 'data_files/training_data.csv'
data_validation = np.genfromtxt(file_path_validation, delimiter=',', skip_header=1)
data = np.genfromtxt(file_path_training, delimiter=',', skip_header=1)

timestamp_t = data[:,0]
thruster0_t = data[:,1]
thruster1_t = data[:,2]
thruster2_t = data[:,3]
thruster3_t = data[:,4]
thruster4_t = data[:,5]
thruster5_t = data[:,6]
thruster6_t = data[:,7]
thruster7_t = data[:,8]
acc_x_t = data[:,9]
acc_y_t = data[:,10]
acc_z_t = data[:,11]
gyro_x_t = data[:,12]
gyro_y_t = data[:,13]
gyro_z_t = data[:,14]

time_diff = 0
for i in range(1, len(timestamp_t)-1):
    time_diff += timestamp_t[i] - timestamp_t[i-1]

time_diff = time_diff / len(timestamp_t)
ts = time_diff.mean()/1000
print(ts)

plt.figure()
plt.plot(timestamp_t, acc_x_t, label="acc_x")
plt.plot(timestamp_t, acc_y_t, label="acc_y")
plt.plot(timestamp_t, acc_z_t, label="acc_z")
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.title("Acceleration")
plt.xlabel('Time')
plt.ylabel('Amplitude')

output_training = np.array([acc_x_t, acc_y_t, acc_z_t, gyro_x_t, gyro_y_t, gyro_z_t])
input_training = np.array([thruster0_t, thruster1_t, thruster2_t, thruster3_t, thruster4_t, thruster5_t, thruster6_t, thruster7_t])
input_validation = np.array([data_validation[:,1], data_validation[:,2], data_validation[:,3], data_validation[:,4], data_validation[:,5], data_validation[:,6], data_validation[:,7], data_validation[:,8]])
output_validation = np.array([data_validation[:,9], data_validation[:,10], data_validation[:,11], data_validation[:,12], data_validation[:,13], data_validation[:,14]])

modelArx = sysid(output_training, input_training, 'N4SID', tsample=ts)
print("A = ", modelArx.A)
print("B = ", modelArx.B)
print("C = ", modelArx.C)
print("D = ", modelArx.D)

outputNames = ['acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z']
sysSS = ct.StateSpace(modelArx.A, modelArx.B, modelArx.C, modelArx.D, ts)

time, outputEst = ct.forced_response(sysSS, T=np.arange(len(data_validation[:,0])) * ts, U=input_validation)

print("NRMSE acc_x:", (sklearn.metrics.mean_squared_error(output_validation[0], outputEst[0]) ** 0.5) / (output_validation[0].max() - output_validation[0].min()))
print("NRMSE acc_y:", (sklearn.metrics.mean_squared_error(output_validation[1], outputEst[1]) ** 0.5) / (output_validation[1].max() - output_validation[1].min()))
print("NRMSE acc_z:", (sklearn.metrics.mean_squared_error(output_validation[2], outputEst[2]) ** 0.5) / (output_validation[2].max() - output_validation[2].min()))
print("NRMSE gyro_x:", (sklearn.metrics.mean_squared_error(output_validation[3], outputEst[3]) ** 0.5) / (output_validation[3].max() - output_validation[3].min()))
print("NRMSE gyro_y:", (sklearn.metrics.mean_squared_error(output_validation[4], outputEst[4]) ** 0.5) / (output_validation[4].max() - output_validation[4].min()))
print("NRMSE gyro_z:", (sklearn.metrics.mean_squared_error(output_validation[5], outputEst[5]) ** 0.5) / (output_validation[5].max() - output_validation[5].min()))

plt.figure()
for i in range(len(outputNames)):
    plt.subplot(7, 1, i+1)
    plt.plot(time, output_validation[i], 'r--', label='Actual Response')
    plt.plot(time, outputEst[i].T, 'b-', label='Model Response')
    plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    plt.title(f'Response - {outputNames[i]}')
    plt.xlabel('Time (s)')
    plt.ylabel('Amplitude')
    plt.grid(True)

plt.subplot(7, 1, 7)
plt.plot(time, input_validation[0], label="Thruster 1")
plt.plot(time, input_validation[1], label="Thruster 2")
plt.plot(time, input_validation[2], label="Thruster 3")
plt.plot(time, input_validation[3], label="Thruster 4")
plt.plot(time, input_validation[4], label="Thruster 5")
plt.plot(time, input_validation[5], label="Thruster 6")
plt.plot(time, input_validation[6], label="Thruster 7")
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.title("Input")
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
#plt.show()

input_arr = np.array([20, 20, -20, -20, 0, 0, 0, 0])
num_steps = 30000
T = np.arange(num_steps) * sysSS.dt
step_start = 30

U = np.zeros((8, num_steps))
U[:, step_start:] = input_arr.reshape(-1, 1)

_, yout = ct.forced_response(sysSS, T=T, U=U)

output_names = ['acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z']

plt.figure(figsize=(10, 6))
for i, name in enumerate(output_names):
    plt.plot(T, yout[i], label=name)

plt.axvline(x=T[step_start], color='k', linestyle=':', label='Sprang start')
plt.xlabel('Tid (s)')
plt.ylabel('Respons')
plt.title(f'Input: {input_arr.tolist()}')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()