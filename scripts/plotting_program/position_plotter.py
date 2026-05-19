import matplotlib.pyplot as plt
import numpy as np

def lowpass_filter_5tap(data):
    filtered = np.zeros_like(data)
    for i in range(len(data)):
        if i < 2:
            filtered[i] = np.mean(data[:i+1])
        elif i >= len(data) - 2:
            filtered[i] = np.mean(data[i-2:])
        else:
            filtered[i] = np.mean(data[i-2:i+3])
    return filtered

file_path = "data_files/objekt_posisjoner.csv"
data = np.genfromtxt(file_path, delimiter=',', skip_header=1)

timestamp = data[:, 0]
object_id = data[:, 1]
x_pos = data[:, 2]
y_pos = data[:, 3]
z_pos = data[:, 4]
class_id = data[:, 5]
classes = ["Aruco marker", "Structure", "Structure wood", "Tube", "Valve"]

data_0 = [[], [], [], [], []]
data_1 = [[], [], [], [], []]

data_2 = [[], [], [], [], []]
data_3 = [[], [], [], [], []]

for i in range(len(object_id)):
    if object_id[i] == 0:
        data_0[0].append(x_pos[i])
        data_0[1].append(y_pos[i])
        data_0[2].append(z_pos[i])
        data_0[3].append(timestamp[i])
        data_0[4].append(class_id[i])
    elif object_id[i] == 1:
        data_1[0].append(x_pos[i])
        data_1[1].append(y_pos[i])
        data_1[2].append(z_pos[i])
        data_1[3].append(timestamp[i])
        data_1[4].append(class_id[i])
    elif object_id[i] == 2:
        data_2[0].append(x_pos[i])
        data_2[1].append(y_pos[i])
        data_2[2].append(z_pos[i])
        data_2[3].append(timestamp[i])
        data_2[4].append(class_id[i])
    elif object_id[i] == 3:
        data_3[0].append(x_pos[i])
        data_3[1].append(y_pos[i])
        data_3[2].append(z_pos[i])
        data_3[3].append(timestamp[i])
        data_3[4].append(class_id[i])

fig, axes = plt.subplots(2, 2, figsize=(12, 10))
fig.suptitle('Object positions over time', fontsize=16)

if data_0[3]:
    axes[0, 0].scatter(data_0[3], data_0[0], alpha=0.3, s=10, label="X raw")
    axes[0, 0].scatter(data_0[3], data_0[1], alpha=0.3, s=10, label="Y raw")
    axes[0, 0].scatter(data_0[3], data_0[2], alpha=0.3, s=10, label="Z raw")
    
    x_filt = lowpass_filter_5tap(np.array(data_0[0]))
    y_filt = lowpass_filter_5tap(np.array(data_0[1]))
    z_filt = lowpass_filter_5tap(np.array(data_0[2]))
    
    axes[0, 0].plot(data_0[3], x_filt, label="X filtered", linewidth=2)
    axes[0, 0].plot(data_0[3], y_filt, label="Y filtered", linewidth=2)
    axes[0, 0].plot(data_0[3], z_filt, label="Z filtered", linewidth=2)
    
    axes[0, 0].set_xlim(data_0[3][0], data_0[3][-1])
    axes[0, 0].set_xlabel("Timestamp")
    axes[0, 0].set_ylabel("Position (m)")
    axes[0, 0].set_title(classes[int(data_0[4][0])])
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

if data_1[3]:
    axes[0, 1].scatter(data_1[3], data_1[0], alpha=0.3, s=10, label="X raw")
    axes[0, 1].scatter(data_1[3], data_1[1], alpha=0.3, s=10, label="Y raw")
    axes[0, 1].scatter(data_1[3], data_1[2], alpha=0.3, s=10, label="Z raw")
    
    x_filt = lowpass_filter_5tap(np.array(data_1[0]))
    y_filt = lowpass_filter_5tap(np.array(data_1[1]))
    z_filt = lowpass_filter_5tap(np.array(data_1[2]))
    
    axes[0, 1].plot(data_1[3], x_filt, label="X filtered", linewidth=2)
    axes[0, 1].plot(data_1[3], y_filt, label="Y filtered", linewidth=2)
    axes[0, 1].plot(data_1[3], z_filt, label="Z filtered", linewidth=2)
    axes[0, 1].set_xlim(data_0[3][0], data_0[3][-1])
    axes[0, 1].set_xlabel("Timestamp")
    axes[0, 1].set_ylabel("Position (m)")
    axes[0, 1].set_title(classes[int(data_1[4][0])])
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)

if data_2[3]:
    axes[1, 0].scatter(data_2[3], data_2[0], alpha=0.3, s=10, label="X raw")
    axes[1, 0].scatter(data_2[3], data_2[1], alpha=0.3, s=10, label="Y raw")
    axes[1, 0].scatter(data_2[3], data_2[2], alpha=0.3, s=10, label="Z raw")
    
    x_filt = lowpass_filter_5tap(np.array(data_2[0]))
    y_filt = lowpass_filter_5tap(np.array(data_2[1]))
    z_filt = lowpass_filter_5tap(np.array(data_2[2]))
    
    axes[1, 0].plot(data_2[3], x_filt, label="X filtered", linewidth=2)
    axes[1, 0].plot(data_2[3], y_filt, label="Y filtered", linewidth=2)
    axes[1, 0].plot(data_2[3], z_filt, label="Z filtered", linewidth=2)
    
    axes[1, 0].set_xlim(data_0[3][0], data_0[3][-1])
    axes[1, 0].set_xlabel("Timestamp")
    axes[1, 0].set_ylabel("Position (m)")
    axes[1, 0].set_title(classes[int(data_2[4][0])])
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

if data_3[3]:
    axes[1, 1].scatter(data_3[3], data_3[0], alpha=0.3, s=10, label="X raw")
    axes[1, 1].scatter(data_3[3], data_3[1], alpha=0.3, s=10, label="Y raw")
    axes[1, 1].scatter(data_3[3], data_3[2], alpha=0.3, s=10, label="Z raw")
    
    x_filt = lowpass_filter_5tap(np.array(data_3[0]))
    y_filt = lowpass_filter_5tap(np.array(data_3[1]))
    z_filt = lowpass_filter_5tap(np.array(data_3[2]))
    
    axes[1, 1].plot(data_3[3], x_filt, label="X filtered", linewidth=2)
    axes[1, 1].plot(data_3[3], y_filt, label="Y filtered", linewidth=2)
    axes[1, 1].plot(data_3[3], z_filt, label="Z filtered", linewidth=2)
    
    axes[1, 1].set_xlim(data_0[3][0], data_0[3][-1])
    axes[1, 1].set_xlabel("Timestamp")
    axes[1, 1].set_ylabel("Position (m)")
    axes[1, 1].set_title(classes[int(data_3[4][0])])
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

print("Klasse:", classes[int(data_0[4][0])], "Mediana avstand z:", np.median(data_0[2]), "Median avstand y:", np.median(data_0[1]), "Median avstand x:", np.median(data_0[0]), "STD:", np.std(data_0[2]), "Length:", len(data_0[2]))
print("Klasse:", classes[int(data_1[4][0])], "Mediana avstand z:", np.median(data_1[2]), "Median avstand y:", np.median(data_1[1]), "Median avstand x:", np.median(data_1[0]), "STD:", np.std(data_1[2]), "Length:", len(data_1[2]))
print("Klasse:", classes[int(data_2[4][0])], "Mediana avstand z:", np.median(data_2[2]), "Median avstand y:", np.median(data_2[1]), "Median avstand x:", np.median(data_2[0]), "STD:", np.std(data_2[2]), "Length:", len(data_2[2]))
print("Klasse:", classes[int(data_3[4][0])], "Mediana avstand z:", np.median(data_3[2]), "Median avstand y:", np.median(data_3[1]), "Median avstand x:", np.median(data_3[0]), "STD:", np.std(data_3[2]), "Length:", len(data_3[2]))