import cv2 as cv
import numpy as np

#test = np.float32([[1,0,100],[0,1,100]])
#print(test)
"""
Will output
[[  1.   0. 100.]
 [  0.   1. 100.]]
"""

img = cv.imread("bebeq.jpg")
# dimensions = (img.shape[1]*2, img.shape[0]*2)
# resize_area= cv.resize(img, dimensions, interpolation=cv.INTER_AREA)
# resize_linear=cv.resize(img, dimensions, interpolation=cv.INTER_LINEAR)
# resize_cubic =cv.resize(img, dimensions, interpolation=cv.INTER_CUBIC)
# cv.imshow("area",resize_area)
# cv.imshow("linear",resize_linear)
# cv.imshow("cubic",resize_cubic)

def translation(img, x, y):
    transMat = np.float32([[1,0,x],[0,1,y]])
    # Positive x -> shift to the right
    # Positive y -> shift down
    # Negative x -> shift left
    # Negative y -> shift up
    dimensions = (img.shape[1],img.shape[0])
    return cv.warpAffine(img, transMat, dimensions)

translated = translation(img, 100,-100)
cv.imshow("Translated", translated)

def rotate (img, angle, center=None, scale=1):
    (height, width) = img.shape[:2]
    if center == None:
        center = (width//2,height//2)
    dimensions = (width,height)
    angle = cv.getRotationMatrix2D(center, angle, scale)
    return cv.warpAffine(img, angle, dimensions)

rotated = rotate(img, 45)
cv.imshow("Rotated", rotated)

flipped = cv.flip(img, 0)
cv.imshow("Flip 0", flipped)
# 0 = Vertical
# 1 = Horizontal
# -1 = Horizontal and vertical

cv.waitKey(0)