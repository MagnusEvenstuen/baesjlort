import cv2

image = cv2.imread("test_images/aruco_test4.jpg")

lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
l, a, b = cv2.split(lab)
clahe = cv2.createCLAHE(clipLimit=1.0)
l = clahe.apply(l)
lab = cv2.merge((l,a,b))
enhanced = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
cv2.imshow("original", enhanced)
cv2.waitKey(0)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
corners, ids, rejected = detector.detectMarkers(enhanced)

if ids is not None:
    for i in range(len(ids)):
        print("id:", ids[i][0])
        debug = image.copy()
        cv2.aruco.drawDetectedMarkers(debug, rejected)
        cv2.imshow(str(ids[i][0]), debug)
        cv2.waitKey(0)
else:
    debug = image.copy()
    cv2.aruco.drawDetectedMarkers(debug, rejected)
    cv2.imshow("rejected", debug)
    cv2.waitKey(0)