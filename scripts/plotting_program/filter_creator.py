import scipy.signal as sig
import numpy as np
import matplotlib.pyplot as plt
import os


#Optimal filter design, and exporting code are modified versions of code from https://github.com/MagnusEvenstuen/DiggSiggECGProject/blob/main/Filtering.py

def exportFilterCoeffs(filter, filename):
    current_dir = os.getcwd()
    filterLen = len(filter)
    print(f"Fila blir lagret i: {current_dir}")

    with open(filename + '.hpp', 'w') as f:
        #Write header with includeguards
        f.write('#ifndef FILTER_COEFFS_H\n')
        f.write('#define FILTER_COEFFS_H\n\n')
        f.write('#include <cstddef>  // for size_t\n\n')

        f.write(f'// Antall koeffisienter: {filterLen}\n')
        f.write(f'constexpr size_t FILTER_LENGTH = {filterLen};\n\n')

        f.write('constexpr float filter_coeffs[FILTER_LENGTH] = {\n')

        #4 coefissients per line
        for i in range(0, len(filter), 4):
            line_coeffs = filter[i:i + 4]
            coeff_str = ', '.join([f'{c:.10f}f' for c in line_coeffs])
            if i + 4 < len(filter):
                f.write(f'    {coeff_str},\n')
            else:
                f.write(f'    {coeff_str}\n')

        f.write('};\n\n')
        f.write('#endif // FILTER_COEFFS_H\n')

    print(f"Filterkoeffisienter eksportert til {filename}.hpp")

def optimalFilter(filterLength, edges, weights, fs):
    lp = sig.remez(
        numtaps=filterLength,
        bands=edges,
        desired=[1, 0],
        weight=weights,
        fs=fs
    )
    lp = lp*np.hamming(filterLength)
    exportFilterCoeffs(lp, "filter_coeffs_lowpass")
    #Plotte kode fra ChatGPT
    w, h = sig.freqz(lp, worN=4096, fs=fs)
    plt.figure(figsize=(10, 6))
    # Amplituderespons (dB)
    plt.subplot(2, 1, 1)
    plt.plot(w, 20 * np.log10(np.abs(h)))
    plt.title("Frekvensrespons")
    plt.ylabel("Amplitude [dB]")
    plt.grid(True)
    # Faserespons
    plt.subplot(2, 1, 2)
    plt.plot(w, np.unwrap(np.angle(h)) * 180 / np.pi)
    plt.ylabel("Fase [grader]")
    plt.xlabel("Frekvens [Hz]")
    plt.grid(True)
    plt.tight_layout()
    plt.show()
    return lp

file_path = 'data_files/sensor_data.csv'
data = np.genfromtxt(file_path, delimiter=',', skip_header=1)

timestamp = data[:, 0]
acc_x = data[:, 1]
acc_y = data[:, 2]
acc_z = data[:, 3]

correct_acc_x = data[:, 10]
correct_acc_y = data[:, 11]
correct_acc_z = data[:, 12]

lp = optimalFilter(21, [0, 5, 6, 50], [3, 1], 100)

acc_x_lp = np.convolve(acc_x, lp, mode="same")
acc_y_lp = np.convolve(acc_y, lp, mode="same")
acc_z_lp = np.convolve(acc_z, lp, mode="same")

plt.figure()
plt.title("Acceleration")
plt.plot(timestamp, acc_x, label="acceleration_x")
plt.plot(timestamp, acc_y, label="acceleration_y")
plt.plot(timestamp, acc_z, label="acceleration_z")
plt.plot(timestamp, acc_x_lp, label="lp_acceleration_x")
plt.plot(timestamp, acc_y_lp, label="lp_acceleration_y")
plt.plot(timestamp, acc_z_lp, label="lp_acceleration_z")
plt.plot(timestamp, correct_acc_x, label="correct_acceleration_x", linestyle=":")
plt.plot(timestamp, correct_acc_y, label="correct_acceleration_y", linestyle=":")
plt.plot(timestamp, correct_acc_z, label="correct_acceleration_z", linestyle=":")
plt.xlabel("Time")
plt.ylabel("Acceleration (m/s^2)")
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
plt.show()