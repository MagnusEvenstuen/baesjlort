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
encoded_bytes = jpegls_encode(image, level=1.0)  # level=1 for lossless
print(time.time()-start)
print(len(encoded_bytes))
#decoded_image = jpegls_decode(encoded_bytes) #to decode