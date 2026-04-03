import cv2 as cv

image = cv.imread("bebeq.jpg")

image = cv.resize(image, (image.shape[1]*3,image.shape[0]*3))

bw_img = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

blur=cv.GaussianBlur(image, (3,3),cv.BORDER_DEFAULT)

edge_cascade = cv.Canny(image, 125,175)
edge_cascade2 = cv.Canny(blur, 125,175)

dialate = cv.dilate(image, (7,7), iterations=1)

erode = cv.erode(image, (3,3), iterations=1) #Opposite of dialate

resized = cv.resize(image, (500, 500), interpolation=cv.INTER_AREA)
#INTER_CUBIC is slower but has the best result compared to INTER_LINEAR or INTER_AREA

cropped = image[100:200,100:200]

cv.imshow("Ori", image)
cv.imshow("BW", bw_img)
cv.imshow("Blur", blur)
# cv.imshow("Edge", edge_cascade)
# cv.imshow("Edge2", edge_cascade2)
# cv.imshow("Dialation", dialate)
# cv.imshow("Erode", erode)
# cv.imshow("Resized", resized)
# cv.imshow("Cropped", cropped)

cv.waitKey(0)