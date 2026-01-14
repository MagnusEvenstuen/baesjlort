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
    bp = sig.remez(
        numtaps=filterLength,
        bands=edges,
        desired=[1, 0],
        weight=weights,
        fs=fs
    )
    bp = bp*np.hamming(filterLength)
    exportFilterCoeffs(bp, "filter_coeffs_lowpass")
    #Plotte kode fra ChatGPT
    w, h = sig.freqz(bp, worN=4096, fs=fs)
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

optimalFilter(21, [0, 5, 6, 50], [3, 1], 100)