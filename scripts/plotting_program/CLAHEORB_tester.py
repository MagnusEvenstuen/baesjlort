import cv2

file_path = 'test_images/test_light_towards.png'
image = cv2.imread(file_path)
gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

lab_image = cv2.cvtColor(image, cv2.COLOR_BGR2Lab)
orb = cv2.ORB_create(nfeatures=10000)
l, a, b = cv2.split(lab_image)
keypoints_original, descriptors_original = orb.detectAndCompute(image, None)
keypoints_gray, descriptors_gray = orb.detectAndCompute(gray_image, None)

clahe = cv2.createCLAHE(clipLimit=1.0)
clahe = clahe.apply(l)
lab_clahe = cv2.merge([clahe, a, b])
result_image_clahe = cv2.cvtColor(lab_clahe, cv2.COLOR_Lab2BGR)
keypoints_clahe, descriptors_clahe = orb.detectAndCompute(result_image_clahe, None)

result_image_original = cv2.cvtColor(image, cv2.COLOR_Lab2BGR)

image_with_keypoints_clahe = cv2.drawKeypoints(result_image_clahe, keypoints_clahe, None, color=(0, 255, 0))
image_with_keypoints_original = cv2.drawKeypoints(image, keypoints_original, None, color=(0, 255, 0))
image_with_keypoints_gray = cv2.drawKeypoints(gray_image, keypoints_gray, None, color=(0, 255, 0))

print("Keypoints CLAHE", len(keypoints_clahe), "Keypoints Original", len(keypoints_original), "Keypoints Gray", len(keypoints_gray))

cv2.imshow('Keypoints with CLAHE', image_with_keypoints_clahe)
cv2.imshow('Keypoints without CLAHE', image_with_keypoints_original)
cv2.waitKey(0)
cv2.destroyAllWindows()