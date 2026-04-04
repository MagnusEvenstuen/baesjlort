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
model = YOLO('YOLO_models/best_subsea_test.pt')
#Camera parameters from rectification
focal_length = 538.78453
baseline_x = 0.0438
orb = cv2.ORB_create(
    nfeatures=1000,      #Default 500, more features = more chances to match
    scaleFactor=1.1,     #Default 1.2, finer scale pyramid
    nlevels=12,          #Default 8, more pyramid levels
    edgeThreshold=10,    #Default 31, detect closer to edges
    fastThreshold=10     #Default 20, lower = detects lower-contrast corners
)
matcher_orb = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
clahe = cv2.createCLAHE(clipLimit=4.0, tileGridSize=(4, 4))

#Fuctions from ROS2 node
def save_bounding_box(results, classes, pos_x, pos_y, boxes, image):
    centers = []
    for i in range(len(results[0].boxes)):
        box = results[0].boxes[i]
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        center = ((int((x1 + x2) / 2), int((y1 + y2) / 2)))

        classes.append(int(box.cls[0]))
        pos_x[0].append(int(x1))
        pos_x[1].append(int(x2))
        pos_y[0].append(int(y1))
        pos_y[1].append(int(y2))
        boxes.append(image[int(y1):int(y2), int(x1):int(x2)])
        centers.append(center)
    return centers

def orb_calculate_depth(left_box, right_box, left_box_left, right_box_left):
        #Does CLAHE and turns image back to BGR
        left_box = cv2.cvtColor(left_box, cv2.COLOR_BGR2Lab)
        right_box = cv2.cvtColor(right_box, cv2.COLOR_BGR2Lab)
        ll, al, bl = cv2.split(left_box)
        lr, ar, br = cv2.split(right_box)
        left_box = cv2.merge((clahe.apply(ll), al, bl))
        right_box = cv2.merge((clahe.apply(lr), ar, br))
        left_box = cv2.cvtColor(left_box, cv2.COLOR_Lab2BGR)
        right_box = cv2.cvtColor(right_box, cv2.COLOR_Lab2BGR)
        cv2.waitKey(0)

        keypoints_left, descriptors_left = orb.detectAndCompute(cv2.cvtColor(left_box, cv2.COLOR_BGR2GRAY), None)       #Sets the colour to grey, and does orb detection on the images
        keypoints_right, descriptors_right = orb.detectAndCompute(cv2.cvtColor(right_box, cv2.COLOR_BGR2GRAY), None)

        if descriptors_left is None or descriptors_right is None:
            print("YAR")
            return None

        matches = matcher_orb.match(descriptors_left, descriptors_right)
        point_disparity = []
        #Calculates the distance between the matches
        for match in matches:
            left_x, left_y = keypoints_left[match.queryIdx].pt
            right_x, right_y = keypoints_right[match.trainIdx].pt

            disparity = (left_x + left_box_left) - (right_x + right_box_left)
            if disparity > 0:   #Only positive disparity is valid, since left camera is to the left of the right camera
                point_disparity.append(disparity)

        if not point_disparity:
            return None
        median_disparity = np.median(point_disparity)
        depth = (baseline_x * focal_length)/median_disparity       #Equation from https://www.youtube.com/watch?v=hUVyDabn1Mg&list=PL2zRqk16wsdoCCLpou-dGo7QQNks1Ppzo&index=6 at 10:20
        return depth

def calculate_depth(image, left_classes, left_pos_x, left_pos_y, left_boxes, right_classes, right_pos_x, right_pos_y, right_boxes):
        if not left_classes or not right_classes or focal_length is None:     #If nothing detected, nothing to calculate depth on
            cv2.imshow('YOLO deteksjon med dybde', image)
            cv2.waitKey(1)
            return

        #Zips information to create one iterable list (those 2 liens are generated using ChatGPT)
        left_objects = [(class_number, (left_pos_x[0][j], left_pos_x[1][j]), (left_pos_y[0][j], left_pos_y[1][j]), j)
                        for j, class_number in enumerate(left_classes)]
        right_objects = [(class_number, (right_pos_x[0][j], right_pos_x[1][j]), (right_pos_y[0][j], right_pos_y[1][j]), j)
                        for j, class_number in enumerate(right_classes)]
        left_objects.sort(key=lambda obj: (obj[2]))
        right_objects.sort(key=lambda obj: (obj[2]))
        left_sorted = [[], [], [], [], []]
        right_sorted = [[], [], [], [], []]

        for obj in left_objects:
            class_number = obj[0]
            left_sorted[class_number].append(obj)
        for obj in right_objects:
             class_number = obj[0]
             right_sorted[class_number].append(obj)

        depth = 0
        for j in range(len(left_sorted)):
            for i in range(min(len(left_sorted[j]), len(right_sorted[j]))):
                left_class, left_x, left_y, left_idx = left_sorted[j][i]
                right_class, right_x, right_y, right_idx = right_sorted[j][i]

                if left_boxes[left_idx].shape[0] < 200 or right_boxes[right_idx].shape[0] < 200:
                    depth = (baseline_x * focal_length) / abs(left_x[0] - right_x[0])
                    print("HER", left_class)
                else:
                    depth = orb_calculate_depth(left_boxes[left_idx], right_boxes[right_idx], left_x[0], right_x[0])
                if depth is not None:
                    #Draw depth info on image (By ChatGPT) (for visualization)
                    center_x = (left_x[0] + left_x[1]) // 2
                    center_y = (left_y[0] + left_y[1]) // 2
                    cv2.putText(image, 
                            f"{depth:.2f}m", 
                            (center_x, center_y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 
                            0.5, 
                            (0, 0, 255), 
                            2)

        cv2.imshow('YOLO deteksjon med dybde', image)
        cv2.waitKey(0)

def process_image(left_image, right_image):
        right_pos_x = [[], []]
        right_pos_y = [[], []]
        right_classes = []
        left_pos_x = [[], []]
        left_pos_y = [[], []]
        left_classes = []
        left_boxes = []
        right_boxes = []

        results_left = model(left_image, conf=0.3)        #Runs the YOLO algorithm. conf is how confident the model has to be to mark the point
        results_right = model(right_image, conf=0.3)
        

        save_bounding_box(results_left, left_classes, left_pos_x, left_pos_y, left_boxes, left_image)
        save_bounding_box(results_right, right_classes, right_pos_x, right_pos_y, right_boxes, right_image)

        calculate_depth(left_image, left_classes, left_pos_x, left_pos_y, left_boxes, right_classes, right_pos_x, right_pos_y, right_boxes)

image_left = cv2.imread("/home/gud/Skole/baesjlort/scripts/plotting_program/test_images/distance_accuracy_test_images/subsea_images/left_subsea2.png")
image_right = cv2.imread("/home/gud/Skole/baesjlort/scripts/plotting_program/test_images/distance_accuracy_test_images/subsea_images/right_subsea2.png")
process_image(image_right, image_left)