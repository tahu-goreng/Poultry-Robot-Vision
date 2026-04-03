import cv2 as cv
import numpy as np

dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)

kode_marker = 1
markerImage = np.zeros((200,200,1), dtype="uint8")

cv.aruco.generateImageMarker(dictionary, kode_marker, 200, markerImage, 1)

cv.imwrite(f"marker_img{kode_marker}.png", markerImage)