import matplotlib.pyplot as plt
import numpy as np

file_path = "data_files/objekt_posisjoner.csv"
data = np.genfromtxt(file_path, delimiter=',', skip_header=1)

timestamp = data[:, 0]
object_id = data[:, 1]
x_pos = data[:, 2]
y_pos = data[:, 3]
z_pos = data[:, 4]

data_0 = [[], [], [], []]
data_1 = [[], [], [], []]
data_2 = [[], [], [], []]
data_3 = [[], [], [], []]

for i in range(len(object_id)):
    if object_id[i] == 0:
        data_0[0].append(x_pos[i])
        data_0[1].append(y_pos[i])
        data_0[2].append(z_pos[i])
        data_0[3].append(timestamp[i])
    elif object_id[i] == 4:
        data_1[0].append(x_pos[i])
        data_1[1].append(y_pos[i])
        data_1[2].append(z_pos[i])
        data_1[3].append(timestamp[i])
    elif object_id[i] == 1:
        data_2[0].append(x_pos[i])
        data_2[1].append(y_pos[i])
        data_2[2].append(z_pos[i])
        data_2[3].append(timestamp[i])
    elif object_id[i] == 3:
        data_3[0].append(x_pos[i])
        data_3[1].append(y_pos[i])
        data_3[2].append(z_pos[i])
        data_3[3].append(timestamp[i])

fig, axes = plt.subplots(2, 2, figsize=(12, 10))
fig.suptitle('Object positions over time', fontsize=16)

if data_0[3]:
    axes[0, 0].scatter(data_0[3], data_0[0], label="X position", linewidth=2)
    axes[0, 0].scatter(data_0[3], data_0[1], label="Y position", linewidth=2)
    axes[0, 0].scatter(data_0[3], data_0[2], label="Z position", linewidth=2)
    axes[0, 0].set_xlim(data_0[3][0], data_0[3][-1])
    axes[0, 0].set_xlabel("Timestamp")
    axes[0, 0].set_ylabel("Position (m)")
    axes[0, 0].set_title("Structure Wood")
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

if data_1[3]:
    axes[0, 1].scatter(data_1[3], data_1[0], label="X position", linewidth=2)
    axes[0, 1].scatter(data_1[3], data_1[1], label="Y position", linewidth=2)
    axes[0, 1].scatter(data_1[3], data_1[2], label="Z position", linewidth=2)
    axes[0, 1].set_xlim(data_0[3][0], data_0[3][-1])
    axes[0, 1].set_xlabel("Timestamp")
    axes[0, 1].set_ylabel("Position (m)")
    axes[0, 1].set_title("Aruco Marker 1")
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)

if data_2[3]:
    axes[1, 0].scatter(data_2[3], data_2[0], label="X position", linewidth=2)
    axes[1, 0].scatter(data_2[3], data_2[1], label="Y position", linewidth=2)
    axes[1, 0].scatter(data_2[3], data_2[2], label="Z position", linewidth=2)
    axes[1, 0].set_xlim(data_0[3][0], data_0[3][-1])
    axes[1, 0].set_xlabel("Timestamp")
    axes[1, 0].set_ylabel("Position (m)")
    axes[1, 0].set_title("Valve")
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

if data_3[3]:
    axes[1, 1].scatter(data_3[3], data_3[0], label="X position", linewidth=2)
    axes[1, 1].scatter(data_3[3], data_3[1], label="Y position", linewidth=2)
    axes[1, 1].scatter(data_3[3], data_3[2], label="Z position", linewidth=2)
    axes[1, 1].set_xlim(data_0[3][0], data_0[3][-1])
    axes[1, 1].set_xlabel("Timestamp")
    axes[1, 1].set_ylabel("Position (m)")
    axes[1, 1].set_title("Aruco Marker 2")
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)

plt.tight_layout()
plt.show()