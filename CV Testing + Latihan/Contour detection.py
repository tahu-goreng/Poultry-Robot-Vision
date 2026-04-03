import cv2 as cv
import numpy as np

image = cv.imread("bebeq.jpg")
image = cv.resize(image, (image.shape[1]*3,image.shape[0]*3), interpolation=cv.INTER_CUBIC)

bw = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

cv.imshow("Black White", image)

canny = cv.Canny(image, 125, 175)
cv.imshow("Canny", canny)

ret, thresh = cv.threshold(bw, 125,255, cv.THRESH_BINARY)
cv.imshow("Threshold", thresh)

contours, hierarchy= cv.findContours(bw, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
print(len(contours))

blank = np.zeros(image.shape, dtype="uint8")

cv.drawContours(blank, contours, -1, (0,0,255), 1)
cv.imshow("Contour", blank)

cv.waitKey(0)