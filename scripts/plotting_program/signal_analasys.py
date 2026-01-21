import numpy as np
import matplotlib.pyplot as plt

file_path = 'data_files/sensor_data.csv'
data = np.genfromtxt(file_path, delimiter=',', skip_header=1)

acc_x = data[:, 1]
acc_y = data[:, 2]
acc_z = data[:, 3]

correct_acc_x = data[:, 10]
correct_acc_y = data[:, 11]
correct_acc_z = data[:, 12]

sample_rate = 100
freqs = np.fft.fftfreq(len(acc_x), 1.0/sample_rate)

acc_x_freq = np.fft.fft(acc_x)
acc_y_freq = np.fft.fft(acc_y)
acc_z_freq = np.fft.fft(acc_z)

correct_acc_x_freq = np.fft.fft(correct_acc_x)
correct_acc_y_freq = np.fft.fft(correct_acc_y)
correct_acc_z_freq = np.fft.fft(correct_acc_z)

plt.figure()
plt.plot(freqs[:len(acc_x)//2], np.abs(acc_x_freq[:len(acc_x)//2]), label='acc_x')
plt.plot(freqs[:len(acc_x)//2], np.abs(acc_y_freq[:len(acc_x)//2]), label='acc_y')
plt.plot(freqs[:len(acc_x)//2], np.abs(acc_z_freq[:len(acc_x)//2]), label='acc_z')
plt.xlabel("Frequency (Hz)")
plt.ylabel("Magnuitude")
plt.title("IMU Acceleration")
plt.legend()

plt.figure()
plt.plot(freqs[:len(acc_x)//2], np.abs(correct_acc_x_freq[:len(acc_x)//2]), label='acc_x')
plt.plot(freqs[:len(acc_x)//2], np.abs(correct_acc_y_freq[:len(acc_x)//2]), label='acc_y')
plt.plot(freqs[:len(acc_x)//2], np.abs(correct_acc_z_freq[:len(acc_x)//2]), label='acc_z')
plt.xlabel("Frequency (Hz)")
plt.ylabel("Magnuitude")
plt.title("Correct Acceleration")
plt.legend()

plt.show()