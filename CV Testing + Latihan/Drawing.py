import cv2 as cv
import numpy as np

blank_image = np.zeros((500,500), dtype='uint8')

capture = cv.imread("test_img.png")

cv.imshow("Blank", blank_image)

cv.waitKey(0)