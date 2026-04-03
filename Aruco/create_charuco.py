import cv2 as cv
import numpy as np

aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_50)

board = cv.aruco.CharucoBoard((7,5), 0.04, 0.03, aruco_dict)

board_img = board.generateImage((2000, 1400), marginSize=50)
cv.imwrite("ChAruCo_7x5.png", board_img)