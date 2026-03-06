import cv2
import numpy as np
from ultralytics import YOLO
import torch
from pathlib import Path
import re

torch.cuda.is_available = lambda: False         #Set true if gpu is available, works fine without.

#Camera and model information. Camera parameters come from calibration
model = YOLO('YOLO_models/yolo_common_objects.pt')
#Camera parameters from rectification
focal_length = 538.78453
baseline_x = 0.0438
sift = cv2.SIFT_create()
orb = cv2.ORB_create()
matcher_orb = cv2.BFMatcher(cv2.NORM_HAMMING)
matcher_sift = cv2.BFMatcher(cv2.NORM_L2)
clahe = cv2.createCLAHE(clipLimit=0.0)
class_number = 76       #Class number of chair, which is used in this test (banana 46, scissors 76)

#Fuctions from ROS2 node
def save_bounding_box(results, classes, pos_x, pos_y, boxes, image):
    for i in range(len(results[0].boxes)):
        box = results[0].boxes[i]
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
        cv2.circle(image, center, 5, (0, 0, 255), -1)

        classes.append(int(box.cls[0]))
        pos_x[0].append(int(x1))
        pos_x[1].append(int(x2))
        pos_y[0].append(int(y1))
        pos_y[1].append(int(y2))
        boxes.append(image[int(y1):int(y2), int(x1):int(x2)])

def orb_calculate_depth(left_box, right_box, left_box_left, right_box_left,
                        baseline, focal_length, detection_type, matcher, clahe):
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

#This function is made by ChatGPT
def extract_number(filename):
    return int(re.findall(r'\d+', filename)[-1])

folder = Path("test_images/distance_accuracy_test_images/scissors_0.5meter")
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
distances_sift_median = []
distances_sift_average = []

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

    save_bounding_box(results_left, left_classes, left_pos_x, left_pos_y, left_boxes, left_image)
    save_bounding_box(results_right, right_classes, right_pos_x, right_pos_y, right_boxes, right_image)

    #56 should be chair used in this test
    if class_number not in left_classes or class_number not in right_classes:
        print("Detection went to shit, can't find place to sit :-(")
        for i in range(len(left_classes)):
            print(left_classes)
        continue

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
            orb_result = orb_calculate_depth(left_boxes[k], right_boxes[k],
                                             left_x[0], right_x[0],
                                             baseline_x, focal_length,
                                             orb, matcher_orb, clahe)
            sift_result = orb_calculate_depth(left_boxes[k], right_boxes[k],
                                              left_x[0], right_x[0],
                                              baseline_x, focal_length,
                                              sift, matcher_sift, clahe)

            if orb_result is not None:
                orb_depth_median, orb_depth_average = orb_result
                distances_orb_median.append(orb_depth_median)
                distances_orb_average.append(orb_depth_average)

            if sift_result is not None:
                sift_depth_median, sift_depth_average = sift_result
                distances_sift_median.append(sift_depth_median)
                distances_sift_average.append(sift_depth_average)
    cv2.imshow(left_file, left_image)


#Takes the average between all the measurements
print(distances_orb_average)
print("Average dist orb average:", np.average(distances_orb_average))
print("Average dist orb median:", np.average(distances_orb_median))
print("Average dist sift average:", np.average(distances_sift_average))
print("Average dist sift median:", np.average(distances_sift_median))
print("Standard deviation orb average:", np.std(distances_orb_average))
print("Standard deviation orb median:", np.std(distances_orb_median))
print("Standard deviation sift average:", np.std(distances_sift_average))
print("Standard deviation sift median:", np.std(distances_sift_median))
print("Minimum orb average:", np.min(distances_orb_average))
print("Minimum orb median:", np.min(distances_orb_median))
print("Maximum orb average:", np.max(distances_orb_average))
print("Maximum orb median:", np.max(distances_orb_median))
print("Minimum sift average:", np.min(distances_sift_average))
print("Minimum sift median:", np.min(distances_sift_median))
print("Maximum sift average:", np.max(distances_sift_average))
print("Maximum sift median:", np.max(distances_sift_median))