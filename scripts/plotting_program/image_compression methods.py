import pywt
import cv2
import numpy as np
import time
import qoi
from imagecodecs import jpegls_encode, jpegls_decode, jpegxs_encode

#Raw
image = cv2.imread("test_images/test_structure.png")
print(image.nbytes)

#Wavelet transform
#Code source https://www.youtube.com/watch?v=eJLF9HeZA8I
start = time.time()
w = "db1"
coefficents = pywt.wavedec2(image, w)
print(time.time()-start)
print(sum(arr.nbytes for arr in coefficents if isinstance(arr, np.ndarray)))

#QOI encoding
start = time.time()
encoded_bytes = qoi.encode(image)
print(time.time()-start)
print(len(encoded_bytes))
#decoded_image = qoi.decode(encoded_bytes) #to decode back to imag

#JPEG-LS (based on wavelet filter https://en.wikipedia.org/wiki/Lossless_JPEG)
start = time.time()
image = cv2.imread("test_images/test_structure.png")
encoded_bytes = jpegls_encode(image, level=1.0)  # level=1 for lossless
#decoded_image = jpegls_decode(encoded_bytes) #to decode
print(time.time()-start)
print(len(encoded_bytes))

start = time.time()
image = cv2.imread("test_images/test_structure.png")
encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
_, encoded_bytes = cv2.imencode('.jpg', image, encode_param)
decoded_image = cv2.imdecode(encoded_bytes, cv2.IMREAD_COLOR)
print(time.time()-start)
print(len(encoded_bytes))

cv2.imshow("image", decoded_image)
cv2.waitKey(0)