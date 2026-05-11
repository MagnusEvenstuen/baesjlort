import pandas as pd
import numpy as np

def getDiff(col):
    return (np.array(col[1:]) - np.array(col[:-1])) * 1e-6

compressedDf = pd.read_csv("compressed.csv")
cpuDf = pd.read_csv("cpu.csv")
gpuDf = pd.read_csv("gpu.csv")

compressedDf = pd.DataFrame({ "cam_left_stamp": getDiff(compressedDf["cam_left_stamp"]),
                              "cam_right_stamp": getDiff(compressedDf["cam_right_stamp"]) })

cpuDf = pd.DataFrame({ "cam_left_stamp": getDiff(cpuDf["cam_left_stamp"]),
                       "cam_right_stamp": getDiff(cpuDf["cam_right_stamp"]) })

gpuDf = pd.DataFrame({ "cam_left_stamp": getDiff(gpuDf["cam_left_stamp"]),
                       "cam_right_stamp": getDiff(gpuDf["cam_right_stamp"]) })

compressedDf.to_csv("compressed_diff.csv", index=False)
cpuDf.to_csv("cpu_diff.csv", index=False)
gpuDf.to_csv("gpu_diff.csv", index=False)
