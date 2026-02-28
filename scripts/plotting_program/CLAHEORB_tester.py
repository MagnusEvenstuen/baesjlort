import cv2

file_path = 'test_images/test_noisy_tube.png'
image = cv2.imread(file_path)

lab_image = cv2.cvtColor(image, cv2.COLOR_BGR2Lab)

l, a, b = cv2.split(lab_image)

clahe = cv2.createCLAHE(clipLimit=4.0)
clahe = clahe.apply(l)

lab_clahe = cv2.merge([clahe, a, b])

result_image = cv2.cvtColor(lab_clahe, cv2.COLOR_Lab2BGR)
orb = cv2.ORB_create()
keypoints, descriptors = orb.detectAndCompute(result_image, None)

image_with_keypoints = cv2.drawKeypoints(result_image, keypoints, None, color=(0, 255, 0))

cv2.imshow('Keypoints med CLAHE', image_with_keypoints)
cv2.waitKey(0)
cv2.destroyAllWindows()