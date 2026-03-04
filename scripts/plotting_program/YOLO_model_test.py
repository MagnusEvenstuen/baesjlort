import cv2
from ultralytics import YOLO
import time
import torch
import numpy as np

torch.cuda.is_available = lambda: False

model = YOLO('YOLO_models/yolo8s.pt')
image1 = cv2.imread("test_images/image_only_noise.png")
image2 = cv2.imread("test_images/low_light_low_feature.png")
image3 = cv2.imread("test_images/test_light_towards.png")
image4 = cv2.imread("test_images/test_noisy_tube.png")
image5 = cv2.imread("test_images/test_structure.png")
images = [image1, image2, image3, image4, image5]

time_now = time.time()

for i in range(200):
    for j in range(len(images)):
        image = images[j]
        model(image)

print("Time taken: ", time.time() - time_now)