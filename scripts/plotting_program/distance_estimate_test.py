import cv2
import numpy as np
from ultralytics import YOLO
import torch
from pathlib import Path
import re
import time
import matplotlib.pyplot as plt

torch.cuda.is_available = lambda: False         #Set true if gpu is available, works fine without.

#Camera and model information. Camera parameters come from calibration
model = YOLO('YOLO_models/yolo_common_objects.pt')
#Camera parameters from rectification
focal_length = 538.78453
baseline_x = 0.0438
sift = cv2.SIFT_create()
orb = cv2.ORB_create()
akaze = cv2.AKAZE_create()
brisk = cv2.BRISK_create()
matcher_orb = cv2.BFMatcher(cv2.NORM_HAMMING)
matcher_sift = cv2.BFMatcher(cv2.NORM_L2)
clahe = cv2.createCLAHE(clipLimit=0.0)
stereo = cv2.StereoSGBM_create(
    minDisparity=0,
    numDisparities=128,
    blockSize=7,
    P1=8 * 3 * 7**2,
    P2=32 * 3 * 7**2,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
)
class_number = 76       #Class number of chair, which is used in this test (banana 46, scissors 76, chair 56)

#Fuctions from ROS2 node
def save_bounding_box(results, classes, pos_x, pos_y, boxes, image):
    centers = []
    for i in range(len(results[0].boxes)):
        box = results[0].boxes[i]
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        center = ((int((x1 + x2) / 2), int((y1 + y2) / 2)))
        cv2.circle(image, center, 5, (0, 0, 255), -1)

        classes.append(int(box.cls[0]))
        pos_x[0].append(int(x1))
        pos_x[1].append(int(x2))
        pos_y[0].append(int(y1))
        pos_y[1].append(int(y2))
        boxes.append(image[int(y1):int(y2), int(x1):int(x2)])
        centers.append(center)
    return centers

def orb_calculate_depth(left_box, right_box, left_box_left, right_box_left,
                        baseline, focal_length, detection_type, matcher, clahe, use_good_matches):
    left_box = cv2.cvtColor(left_box, cv2.COLOR_BGR2Lab)
    right_box = cv2.cvtColor(right_box, cv2.COLOR_BGR2Lab)
    ll, al, bl = cv2.split(left_box)
    lr, ar, br = cv2.split(right_box)
    left_box = cv2.merge((clahe.apply(ll), al, bl))
    right_box = cv2.merge((clahe.apply(lr), ar, br))
    left_box = cv2.cvtColor(left_box, cv2.COLOR_Lab2BGR)
    right_box = cv2.cvtColor(right_box, cv2.COLOR_Lab2BGR)

    keypoints_left, descriptors_left = detection_type.detectAndCompute(
        cv2.cvtColor(left_box, cv2.COLOR_BGR2GRAY), None)
    keypoints_right, descriptors_right = detection_type.detectAndCompute(
        cv2.cvtColor(right_box, cv2.COLOR_BGR2GRAY), None)

    if descriptors_left is None or descriptors_right is None:
        return None

    matches = matcher.match(descriptors_left, descriptors_right)
    if use_good_matches:
        matches = sorted(matches, key=lambda x: x.distance)
        matches = matches[:30]
    point_disparity = []
    for match in matches:
        left_x, left_y = keypoints_left[match.queryIdx].pt
        right_x, right_y = keypoints_right[match.trainIdx].pt

        disparity = (left_x + left_box_left) - (right_x + right_box_left)
        if disparity > 0:
            point_disparity.append(disparity)

    if not point_disparity:
        return None
    median_disparity = np.median(point_disparity)
    average_disparity = np.average(point_disparity)
    depth_median = (baseline * focal_length) / median_disparity
    depth_average = (baseline * focal_length) / average_disparity
    return depth_median, depth_average

def SGBM_depth(left_image, right_image, x, y, baseline, focal):
    disparity = stereo.compute(left_image, right_image).astype(np.float32) / 16.0

    disp = disparity[int(y-10):int(y+10), int(x-10):int(x+10)]
    disp = disp[disp > 0]

    if disp.size == 0:
        return None

    depth = (baseline * focal) / np.median(disp)
    return depth

#This function is made by ChatGPT
def extract_number(filename):
    return int(re.findall(r'\d+', filename)[-1])

distances_orb_median_all = []
distances_orb_average_all = []
orb_time_all = []
stdev_orb_median_all = []
stdev_orb_average_all = []
distances_sift_median_all = []
distances_sift_average_all = []
sift_time_all = []
stdev_sift_median_all = []
stdev_sift_average_all = []
distances_akaze_median_all = []
distances_akaze_average_all = []
akaze_time_all = []
stdev_akaze_median_all = []
stdev_akaze_average_all = []
distances_brisk_median_all = []
distances_brisk_average_all = []
brisk_time_all = []
stdev_brisk_median_all = []
stdev_brisk_average_all = []
distances_SGBM_all = []
SGBM_time_all = []
stdev_SGBM_all = []
distances_center_difference_all = []
distance_time_all = []
stdev_center_diff_all = []
images_used = []

#folder = Path("test_images/distance_accuracy_test_images/scissors_0.6meter")
files_to_use = ["test_images/distance_accuracy_test_images/scissors_0.2meter", "test_images/distance_accuracy_test_images/scissors_0.3meter", "test_images/distance_accuracy_test_images/scissors_0.4meter", "test_images/distance_accuracy_test_images/scissors_0.5meter", "test_images/distance_accuracy_test_images/scissors_0.6meter"]
#files_to_use = ["test_images/distance_accuracy_test_images/0.57meter_chair", "test_images/distance_accuracy_test_images/1meter_chair", "test_images/distance_accuracy_test_images/2meter_chair", "test_images/distance_accuracy_test_images/3meter_chair", "test_images/distance_accuracy_test_images/5meter_chair"]
true_distances = [0.2, 0.3, 0.4, 0.5, 0.6]
#true_distances = [0.57, 1, 2, 3, 5]
for i in range(len(files_to_use)):
    folder = Path(files_to_use[i])
    files = [f.name for f in folder.iterdir() if f.is_file()]
    left_images = [f for f in files if f.startswith("_left_image_rect_color_")]
    right_images = [f for f in files if f.startswith("_right_image_rect_color_")]

    print(len(left_images))
    print(len(right_images))

    left_pairs = [(extract_number(f), f) for f in left_images]
    right_pairs = [(extract_number(f), f) for f in right_images]

    left_pairs.sort()
    right_pairs.sort()

    matched_pairs = []
    j = 0
    for left_time, left_file in left_pairs:
        while j < len(right_pairs) - 1 and abs(right_pairs[j+1][0] - left_time) < abs(right_pairs[j][0] - left_time):
            j += 1
        matched_pairs.append((left_file, right_pairs[j][1]))

    distances_orb_median = []
    distances_orb_average = []
    orb_time = []
    distances_sift_median = []
    distances_sift_average = []
    sift_time = []
    distances_akaze_median = []
    distances_akaze_average = []
    akaze_time = []
    distances_brisk_median = []
    distances_brisk_average = []
    brisk_time = []
    distances_SGBM = []
    SGBM_time = []
    distances_center_difference = []
    distance_time = []

    #From here most of the code is taken from the YOLO node, and modified to fit here
    for i in range(len(matched_pairs)):
        left_file, right_file = matched_pairs[i]
        left_image = cv2.imread(str(folder / left_file))
        right_image = cv2.imread(str(folder / right_file))
        results_left = model(left_image, conf=0.3)  # Runs the YOLO algorithm. conf is how confident the model has to be to mark the point
        results_right = model(right_image, conf=0.3)

        left_classes = []
        left_pos_x = [[], []]  # [x1, x2]
        left_pos_y = [[], []]  # [y1, y2]
        left_boxes = []

        right_classes = []
        right_pos_x = [[], []]
        right_pos_y = [[], []]
        right_boxes = []

        center_left = save_bounding_box(results_left, left_classes, left_pos_x, left_pos_y, left_boxes, left_image)
        center_right = save_bounding_box(results_right, right_classes, right_pos_x, right_pos_y, right_boxes, right_image)

        if class_number not in left_classes or class_number not in right_classes:
            print("Detection went to shit, can't find place to sit :-(")
            print(left_classes)
            continue

        left_index = left_classes.index(class_number)
        right_index= right_classes.index(class_number)
        center_left = center_left[left_index]
        center_right = center_right[right_index]
        if center_left[0] != center_right[0]:
            start_time = time.time()
            distances_center_difference.append((baseline_x * focal_length) / (center_left[0] - center_right[0]))
            distance_time.append(time.time() - start_time)

        start_time = time.time()
        SGBM_depth_value = SGBM_depth(
            left_image,
            right_image,
            center_left[0],
            center_left[1],
            baseline_x,
            focal_length
        )
        SGBM_time.append(time.time() - start_time)

        if SGBM_depth_value is not None:
            distances_SGBM.append(SGBM_depth_value)

        left_objects = [(class_number,
                         (left_pos_x[0][j], left_pos_x[1][j]),
                         (left_pos_y[0][j], left_pos_y[1][j]))
                        for j, class_number in enumerate(left_classes)]
        right_objects = [(class_number,
                          (right_pos_x[0][j], right_pos_x[1][j]),
                          (right_pos_y[0][j], right_pos_y[1][j]))
                         for j, class_number in enumerate(right_classes)]
        left_objects.sort(key=lambda obj: (obj[0], obj[2]))
        right_objects.sort(key=lambda obj: (obj[0], obj[2]))

        for k in range(min(len(left_objects), len(right_objects))):
            left_class, left_x, left_y = left_objects[k]
            right_class, right_x, right_y = right_objects[k]

            #Only calculate on chairs
            if left_class == right_class and left_class == class_number:
                start_time = time.time()
                orb_result = orb_calculate_depth(left_boxes[k], right_boxes[k],
                                                 left_x[0], right_x[0],
                                                 baseline_x, focal_length,
                                                 orb, matcher_orb, clahe, False)
                orb_time.append(time.time() - start_time)

                start_time = time.time()
                sift_result = orb_calculate_depth(left_boxes[k], right_boxes[k],
                                                  left_x[0], right_x[0],
                                                  baseline_x, focal_length,
                                                  sift, matcher_sift, clahe, False)
                sift_time.append(time.time() - start_time)

                start_time = time.time()
                akaze_result = orb_calculate_depth(left_boxes[k], right_boxes[k],
                                                  left_x[0], right_x[0],
                                                  baseline_x, focal_length,
                                                  akaze, matcher_orb, clahe, False)
                akaze_time.append(time.time() - start_time)

                start_time = time.time()
                brisk_result = orb_calculate_depth(left_boxes[k], right_boxes[k],
                                                  left_x[0], right_x[0],
                                                  baseline_x, focal_length,
                                                  brisk, matcher_orb, clahe, False)
                brisk_time.append(time.time() - start_time)

                if orb_result is not None:
                    orb_depth_median, orb_depth_average = orb_result
                    distances_orb_median.append(orb_depth_median)
                    distances_orb_average.append(orb_depth_average)

                if sift_result is not None:
                    sift_depth_median, sift_depth_average = sift_result
                    distances_sift_median.append(sift_depth_median)
                    distances_sift_average.append(sift_depth_average)

                if akaze_result is not None:
                    akaze_depth_median, akaze_depth_average = akaze_result
                    distances_akaze_median.append(akaze_depth_median)
                    distances_akaze_average.append(akaze_depth_average)

                if brisk_result is not None:
                    brisk_depth_median, brisk_depth_average = brisk_result
                    distances_brisk_median.append(brisk_depth_median)
                    distances_brisk_average.append(brisk_depth_average)

        #cv2.imshow(left_file, left_image)

    print("Images used:", len(distances_orb_average))
    images_used.append(len(distances_orb_average))
    #Takes the average between all the measurements
    print("Average dist orb average:", np.average(distances_orb_average))
    print("Average dist orb median:", np.average(distances_orb_median))
    print("Average dist sift average:", np.average(distances_sift_average))
    print("Average dist sift median:", np.average(distances_sift_median))
    print("Average dist akaze average:", np.average(distances_akaze_average))
    print("Average dist akaze median:", np.average(distances_akaze_median))
    print("Average dist brisk average:", np.average(distances_brisk_average))
    print("Average dist brisk median:", np.average(distances_brisk_median))
    print("Average dist BB center:", np.average(distances_center_difference))
    print("Average SGBM:", np.average(distances_SGBM))
    print("Standard deviation orb average:", np.std(distances_orb_average))
    print("Standard deviation orb median:", np.std(distances_orb_median))
    print("Standard deviation sift average:", np.std(distances_sift_average))
    print("Standard deviation sift median:", np.std(distances_sift_median))
    print("Standard deviation akaze average:", np.std(distances_akaze_average))
    print("Standard deviation akaze median:", np.std(distances_akaze_median))
    print("Standard deviation brisk average:", np.std(distances_brisk_average))
    print("Standard deviation brisk median:", np.std(distances_brisk_median))
    print("Standard deviation BB center:", np.std(distances_center_difference))
    print("Standard deviation SGBM:", np.std(distances_SGBM))
    print("Minimum orb average:", np.min(distances_orb_average))
    print("Minimum orb median:", np.min(distances_orb_median))
    print("Maximum orb average:", np.max(distances_orb_average))
    print("Maximum orb median:", np.max(distances_orb_median))
    print("Minimum akaze average:", np.min(distances_akaze_average))
    print("Minimum akaze median:", np.min(distances_akaze_median))
    print("Maximum akaze average:", np.max(distances_akaze_average))
    print("Maximum akaze median:", np.max(distances_akaze_median))
    print("Minimum brisk average:", np.min(distances_brisk_average))
    print("Minimum brisk median:", np.min(distances_brisk_median))
    print("Maximum brisk average:", np.max(distances_brisk_average))
    print("Maximum brisk median:", np.max(distances_brisk_median))
    print("Minimum BB center:", np.min(distances_center_difference))
    print("Maximum BB center:", np.max(distances_center_difference))
    print("Minimum sift average:", np.min(distances_sift_average))
    print("Minimum sift median:", np.min(distances_sift_median))
    print("Maximum sift average:", np.max(distances_sift_average))
    print("Maximum sift median:", np.max(distances_sift_median))
    print("Minimum SGBM:", np.min(distances_SGBM))
    print("Maximum SGBM:", np.max(distances_SGBM))
    print("Time spent orb:", np.average(orb_time))
    print("Time spent sift:", np.average(sift_time))
    print("Time spent akaze:", np.average(akaze_time))
    print("Time spent brisk:", np.average(brisk_time))
    print("Time spent BB center:", np.average(distance_time))
    print("Time spent SGBM:", np.average(SGBM_time))

    distances_orb_median_all.append(np.average(distances_orb_median))
    distances_orb_average_all.append(np.average(distances_orb_average))
    orb_time_all.append(np.average(orb_time))
    distances_sift_median_all.append(np.average(distances_sift_median))
    distances_sift_average_all.append(np.average(distances_sift_average))
    sift_time_all.append(np.average(sift_time))
    distances_akaze_median_all.append(np.average(distances_akaze_median))
    distances_akaze_average_all.append(np.average(distances_akaze_average))
    akaze_time_all.append(np.average(akaze_time))
    distances_brisk_median_all.append(np.average(distances_brisk_median))
    distances_brisk_average_all.append(np.average(distances_brisk_average))
    brisk_time_all.append(np.average(brisk_time))
    distances_SGBM_all.append(np.average(distances_SGBM))
    SGBM_time_all.append(np.average(SGBM_time))
    distances_center_difference_all.append(np.average(distances_center_difference))
    distance_time_all.append(np.average(distance_time))
    stdev_orb_median_all.append(np.std(distances_orb_median))
    stdev_orb_average_all.append(np.std(distances_orb_average))
    stdev_sift_median_all.append(np.std(distances_sift_median))
    stdev_sift_average_all.append(np.std(distances_sift_average))
    stdev_akaze_median_all.append(np.std(distances_akaze_median))
    stdev_akaze_average_all.append(np.std(distances_akaze_average))
    stdev_brisk_median_all.append(np.std(distances_brisk_median))
    stdev_brisk_average_all.append(np.std(distances_brisk_average))
    stdev_SGBM_all.append(np.std(distances_SGBM))
    stdev_center_diff_all.append(np.std(distances_center_difference))

#All the plotting code below is generated by ChatGPT
print(images_used)
plt.figure(figsize=(12, 8))

# Plot den ideelle linjen (y = x)
plt.plot(true_distances, true_distances, 'k--', linewidth=2, label='Ideell linje (y=x)', alpha=0.7)

# Farger for de ulike metodene
colors = {
    'orb_avg': 'red',
    'orb_med': 'darkred',
    'sift_avg': 'blue',
    'sift_med': 'darkblue',
    'akaze_avg': 'green',
    'akaze_med': 'darkgreen',
    'brisk_avg': 'purple',
    'brisk_med': 'darkviolet',
    'bb_center': 'orange',
    'sgbm': 'brown'
}

plt.errorbar(true_distances, distances_orb_average_all, yerr=stdev_orb_average_all,
             fmt='o-', color=colors['orb_avg'], capsize=5, label='ORB Average', alpha=0.7)
plt.errorbar(true_distances, distances_orb_median_all, yerr=stdev_orb_median_all,
             fmt='s-', color=colors['orb_med'], capsize=5, label='ORB Median', alpha=0.7)

plt.errorbar(true_distances, distances_sift_average_all, yerr=stdev_sift_average_all,
             fmt='o-', color=colors['sift_avg'], capsize=5, label='SIFT Average', alpha=0.7)
plt.errorbar(true_distances, distances_sift_median_all, yerr=stdev_sift_median_all,
             fmt='s-', color=colors['sift_med'], capsize=5, label='SIFT Median', alpha=0.7)

plt.errorbar(true_distances, distances_akaze_average_all, yerr=stdev_akaze_average_all,
             fmt='o-', color=colors['akaze_avg'], capsize=5, label='AKAZE Average', alpha=0.7)
plt.errorbar(true_distances, distances_akaze_median_all, yerr=stdev_akaze_median_all,
             fmt='s-', color=colors['akaze_med'], capsize=5, label='AKAZE Median', alpha=0.7)

plt.errorbar(true_distances, distances_brisk_average_all, yerr=stdev_brisk_average_all,
             fmt='o-', color=colors['brisk_avg'], capsize=5, label='BRISK Average', alpha=0.7)
plt.errorbar(true_distances, distances_brisk_median_all, yerr=stdev_brisk_median_all,
             fmt='s-', color=colors['brisk_med'], capsize=5, label='BRISK Median', alpha=0.7)

plt.errorbar(true_distances, distances_center_difference_all, yerr=stdev_center_diff_all,
             fmt='d-', color=colors['bb_center'], capsize=5, label='Bounding Box Center', alpha=0.7, linewidth=2)

plt.errorbar(true_distances, distances_SGBM_all, yerr=stdev_SGBM_all,
             fmt='^-', color=colors['sgbm'], capsize=5, label='SGBM', alpha=0.7, linewidth=2)

plt.xlabel('True distance (meter)', fontsize=12)
plt.ylabel('Estimated distance (meter)', fontsize=12)
plt.title('Compairison of true vs estimated distance', fontsize=14)
plt.grid(True, alpha=0.3)
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=10)
plt.tight_layout()
plt.show()

fig, axes = plt.subplots(2, 3, figsize=(15, 10))
axes = axes.flatten()

methods = [
    {
        'name': 'ORB',
        'data_avg': distances_orb_average_all,
        'data_med': distances_orb_median_all,
        'stdev_avg': stdev_orb_average_all,
        'stdev_med': stdev_orb_median_all,
        'color_avg': colors['orb_avg'],
        'color_med': colors['orb_med'],
        'marker_avg': 'o',
        'marker_med': 's'
    },
    {
        'name': 'SIFT',
        'data_avg': distances_sift_average_all,
        'data_med': distances_sift_median_all,
        'stdev_avg': stdev_sift_average_all,
        'stdev_med': stdev_sift_median_all,
        'color_avg': colors['sift_avg'],
        'color_med': colors['sift_med'],
        'marker_avg': 'o',
        'marker_med': 's'
    },
    {
        'name': 'AKAZE',
        'data_avg': distances_akaze_average_all,
        'data_med': distances_akaze_median_all,
        'stdev_avg': stdev_akaze_average_all,
        'stdev_med': stdev_akaze_median_all,
        'color_avg': colors['akaze_avg'],
        'color_med': colors['akaze_med'],
        'marker_avg': 'o',
        'marker_med': 's'
    },
    {
        'name': 'BRISK',
        'data_avg': distances_brisk_average_all,
        'data_med': distances_brisk_median_all,
        'stdev_avg': stdev_brisk_average_all,
        'stdev_med': stdev_brisk_median_all,
        'color_avg': colors['brisk_avg'],
        'color_med': colors['brisk_med'],
        'marker_avg': 'o',
        'marker_med': 's'
    },
    {
        'name': 'BB Center',
        'data_avg': distances_center_difference_all,  # BB Center har bare én verdi
        'data_med': None,  # Ingen median for BB Center
        'stdev_avg': stdev_center_diff_all,
        'stdev_med': None,
        'color_avg': colors['bb_center'],
        'color_med': None,
        'marker_avg': 'd',
        'marker_med': None
    },
    {
        'name': 'SGBM',
        'data_avg': distances_SGBM_all,  # SGBM har bare én verdi
        'data_med': None,  # Ingen median for SGBM
        'stdev_avg': stdev_SGBM_all,
        'stdev_med': None,
        'color_avg': colors['sgbm'],
        'color_med': None,
        'marker_avg': '^',
        'marker_med': None
    }
]

for idx, method in enumerate(methods):
    ax = axes[idx]

    ax.plot(true_distances, true_distances, 'k--', label='Ideal', alpha=0.5, linewidth=1.5)

    if method['data_avg'] is not None and len(method['data_avg']) > 0:
        ax.errorbar(true_distances, method['data_avg'],
                    yerr=method['stdev_avg'],
                    fmt=method['marker_avg'] + '-',
                    color=method['color_avg'],
                    capsize=5,
                    label=f'{method["name"]} Average',
                    alpha=0.8,
                    linewidth=2,
                    markersize=8)

    if method['data_med'] is not None and len(method['data_med']) > 0:
        ax.errorbar(true_distances, method['data_med'],
                    yerr=method['stdev_med'],
                    fmt=method['marker_med'] + '--',
                    color=method['color_med'],
                    capsize=5,
                    label=f'{method["name"]} Median',
                    alpha=0.8,
                    linewidth=2,
                    markersize=8)

    ax.set_xlabel('True distance (m)', fontsize=11)
    ax.set_ylabel('Estimated distance (m)', fontsize=11)
    ax.set_title(f'{method["name"]} - Average vs Median', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper left', fontsize=9)

    if method['data_avg'] is not None and len(method['data_avg']) > 0:
        avg_errors = [abs(method['data_avg'][i] - true_distances[i]) for i in range(len(true_distances))]
        avg_error = np.mean(avg_errors)

        stats_text = f'Avg error: {avg_error:.3f}m'

        if method['data_med'] is not None and len(method['data_med']) > 0:
            med_errors = [abs(method['data_med'][i] - true_distances[i]) for i in range(len(true_distances))]
            med_error = np.mean(med_errors)
            stats_text += f'\nMed error: {med_error:.3f}m'

        # Plasser tekstboksen i nedre høyre hjørne
        ax.text(0.95, 0.05, stats_text, transform=ax.transAxes,
                fontsize=9, verticalalignment='bottom', horizontalalignment='right',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

plt.tight_layout()
plt.show()

plt.figure(figsize=(12, 6))
x_pos = np.arange(len(true_distances))
width = 0.12

methods_stdev = [
    ('ORB Avg', stdev_orb_average_all, colors['orb_avg']),
    ('ORB Med', stdev_orb_median_all, colors['orb_med']),
    ('SIFT Avg', stdev_sift_average_all, colors['sift_avg']),
    ('SIFT Med', stdev_sift_median_all, colors['sift_med']),
    ('AKAZE Avg', stdev_akaze_average_all, colors['akaze_avg']),
    ('AKAZE Med', stdev_akaze_median_all, colors['akaze_med']),
    ('BRISK Avg', stdev_brisk_average_all, colors['brisk_avg']),
    ('BRISK Med', stdev_brisk_median_all, colors['brisk_med']),
    ('BB Center', stdev_center_diff_all, colors['bb_center']),
    ('SGBM', stdev_SGBM_all, colors['sgbm'])
]

for i, (name, stdev, color) in enumerate(methods_stdev):
    plt.bar(x_pos + i*width, stdev, width, label=name, color=color, alpha=0.7)

plt.xlabel('True distance (meter)', fontsize=12)
plt.ylabel('Standard deviation (meter)', fontsize=12)
plt.title('Standard deviation in distance estimates', fontsize=14)
plt.xticks(x_pos + width * 4.5, [f'{d}m' for d in true_distances])
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=9)
plt.grid(True, alpha=0.3, axis='y')
plt.tight_layout()
plt.show()

# Plott for tidsbruk
plt.figure(figsize=(10, 6))

times = [
    ('ORB', orb_time_all, colors['orb_avg']),
    ('SIFT', sift_time_all, colors['sift_avg']),
    ('AKAZE', akaze_time_all, colors['akaze_avg']),
    ('BRISK', brisk_time_all, colors['brisk_avg']),
    ('BB Center', distance_time_all, colors['bb_center']),
    ('SGBM', SGBM_time_all, colors['sgbm'])
]

for name, time_data, color in times:
    plt.plot(true_distances, time_data, 'o-', color=color, label=name, linewidth=2, markersize=8)

plt.xlabel('True distance (meter)', fontsize=12)
plt.ylabel('Time spent (seconds)', fontsize=12)
plt.title('Time spent at different distances', fontsize=14)
plt.grid(True, alpha=0.3)
plt.legend()
plt.tight_layout()
plt.show()

# Plott for relativ feil (prosent)
plt.figure(figsize=(12, 8))

# Beregn relativ feil for alle metoder
rel_errors = {}

# ORB
if distances_orb_average_all:
    rel_errors['ORB Avg'] = [abs(distances_orb_average_all[i] - true_distances[i]) / true_distances[i] * 100
                             for i in range(len(true_distances))]
if distances_orb_median_all:
    rel_errors['ORB Med'] = [abs(distances_orb_median_all[i] - true_distances[i]) / true_distances[i] * 100
                             for i in range(len(true_distances))]

# SIFT
if distances_sift_average_all:
    rel_errors['SIFT Avg'] = [abs(distances_sift_average_all[i] - true_distances[i]) / true_distances[i] * 100
                              for i in range(len(true_distances))]
if distances_sift_median_all:
    rel_errors['SIFT Med'] = [abs(distances_sift_median_all[i] - true_distances[i]) / true_distances[i] * 100
                              for i in range(len(true_distances))]

# AKAZE
if distances_akaze_average_all:
    rel_errors['AKAZE Avg'] = [abs(distances_akaze_average_all[i] - true_distances[i]) / true_distances[i] * 100
                               for i in range(len(true_distances))]
if distances_akaze_median_all:
    rel_errors['AKAZE Med'] = [abs(distances_akaze_median_all[i] - true_distances[i]) / true_distances[i] * 100
                               for i in range(len(true_distances))]

# BRISK
if distances_brisk_average_all:
    rel_errors['BRISK Avg'] = [abs(distances_brisk_average_all[i] - true_distances[i]) / true_distances[i] * 100
                               for i in range(len(true_distances))]
if distances_brisk_median_all:
    rel_errors['BRISK Med'] = [abs(distances_brisk_median_all[i] - true_distances[i]) / true_distances[i] * 100
                               for i in range(len(true_distances))]

# BB Center
if distances_center_difference_all:
    rel_errors['BB Center'] = [abs(distances_center_difference_all[i] - true_distances[i]) / true_distances[i] * 100
                               for i in range(len(true_distances))]

# SGBM
if distances_SGBM_all:
    rel_errors['SGBM'] = [abs(distances_SGBM_all[i] - true_distances[i]) / true_distances[i] * 100
                          for i in range(len(true_distances))]

# Plott relativ feil
markers = ['o', 's', '^', 'd', 'v', '*', 'p', 'h', 'x', '+']
colors_list = ['red', 'darkred', 'blue', 'darkblue', 'green', 'darkgreen',
               'purple', 'darkviolet', 'orange', 'brown']

for i, (name, errors) in enumerate(rel_errors.items()):
    plt.plot(true_distances, errors, marker=markers[i % len(markers)],
             linestyle='-', color=colors_list[i % len(colors_list)],
             label=name, linewidth=2, markersize=8, alpha=0.8)

plt.xlabel('True distance (meter)', fontsize=12)
plt.ylabel('Relative error (%)', fontsize=12)
plt.title('Relative error vs distance', fontsize=14)
plt.grid(True, alpha=0.3)
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=9)
plt.tight_layout()
plt.show()

# Ekstra: Stabelplot av relativ feil for å sammenligne metoder
plt.figure(figsize=(14, 6))

# Gruppert bar plot for relativ feil
x = np.arange(len(true_distances))
width = 0.1
offsets = np.linspace(-0.4, 0.4, len(rel_errors))

for i, (name, errors) in enumerate(rel_errors.items()):
    plt.bar(x + offsets[i], errors, width, label=name,
            color=colors_list[i % len(colors_list)], alpha=0.7)

plt.xlabel('True distance (meter)', fontsize=12)
plt.ylabel('Relative error (%)', fontsize=12)
plt.title('Relative error comparison at each distance', fontsize=14)
plt.xticks(x, [f'{d}m' for d in true_distances])
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=9)
plt.grid(True, alpha=0.3, axis='y')
plt.tight_layout()
plt.show()

# Ekstra: Gjennomsnittlig relativ feil for hver metode
plt.figure(figsize=(10, 6))

avg_rel_errors = [np.mean(errors) for errors in rel_errors.values()]
names = list(rel_errors.keys())

bars = plt.bar(range(len(names)), avg_rel_errors, color=colors_list[:len(names)], alpha=0.7)
plt.xlabel('Method', fontsize=12)
plt.ylabel('Average relative error (%)', fontsize=12)
plt.title('Average relative error by method', fontsize=14)
plt.xticks(range(len(names)), names, rotation=45, ha='right')
plt.grid(True, alpha=0.3, axis='y')

# Legg til verdier på stolpene
for i, (bar, val) in enumerate(zip(bars, avg_rel_errors)):
    plt.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
             f'{val:.1f}%', ha='center', va='bottom', fontsize=9)

plt.tight_layout()
plt.show()