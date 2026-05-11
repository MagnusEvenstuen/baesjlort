import pandas as pd
import numpy as np
import matplotlib.pyplot as pp

df = pd.read_csv("compressed_diff.csv")

print("Camera read latency (ms)")
print(df.describe([0.95, 0.975, 0.99]))
print()
df.plot.hist(alpha=0.5)
pp.title("Camera reader")
pp.xlabel("Latency (ms)")
pp.show()

df = pd.read_csv("cpu_diff.csv")

print("CPU latency (ms)")
print(df.describe([0.95, 0.975, 0.99]))
print()
df.plot.hist(alpha=0.5)
pp.title("CPU decoder")
pp.xlabel("Latency (ms)")
pp.show()

df = pd.read_csv("gpu_diff.csv").dropna() 

print("GPU latency (ms)")
print(df.describe([0.95, 0.975, 0.99]))
df.plot.hist(alpha=0.5)
pp.title("GPU decoder")
pp.xlabel("Latency (ms)")
pp.show()
