import matplotlib.pyplot as pp
import pandas as pd
import numpy as np

# def getDiff(df):
    # return (np.array(df.iloc[1:, 1]) - np.array(df.iloc[:-1, 1])) * 1e-9
def getDiff(col):
    return (np.array(col[1:]) - np.array(col[:-1])) * 1e-6

compressedDf = pd.read_csv("compressed.csv")
cpuDf = pd.read_csv("cpu.csv")
gpuDf = pd.read_csv("gpu.csv")


# print(diff)
pp.figure()
pp.suptitle("Compressed image retrieved latency")
ax1 = pp.subplot(1, 2, 1)
pp.title("Left Camera")
pp.ylabel("Milliseconds")
pp.plot(getDiff(compressedDf["cam_left_stamp"]))
pp.subplot(1, 2, 2, sharex=ax1, sharey=ax1)
pp.title("Right Camera")
pp.ylabel("Milliseconds")
pp.plot(getDiff(compressedDf["cam_right_stamp"]))
pp.show()

pp.figure()
pp.suptitle("Cpu image decode latency")
ax1 = pp.subplot(1, 2, 1)
pp.title("Left Camera")
pp.ylabel("Milliseconds")
pp.plot(getDiff(cpuDf["cam_left_stamp"]))
pp.subplot(1, 2, 2, sharex=ax1, sharey=ax1)
pp.title("Right Camera")
pp.ylabel("Milliseconds")
pp.plot(getDiff(cpuDf["cam_right_stamp"]))
pp.show()

pp.figure()
pp.suptitle("Gpu image decode latency")
ax1 = pp.subplot(1, 2, 1)
pp.title("Left Camera")
pp.ylabel("Milliseconds")
pp.plot(getDiff(gpuDf["cam_left_stamp"]))
pp.subplot(1, 2, 2, sharex=ax1, sharey=ax1)
pp.title("Right Camera")
pp.ylabel("Milliseconds")
pp.plot(getDiff(gpuDf["cam_right_stamp"]))
pp.show()
