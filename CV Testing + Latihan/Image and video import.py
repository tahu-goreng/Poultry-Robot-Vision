import cv2 as cv

image = cv.imread("test_img.png")
# video = cv.VideoCapture("IMG_E2872.MOV")
video= cv.VideoCapture(1, cv.CAP_DSHOW)

#Image
# cv.imshow("Test Image", image)
# cv.waitKey(0)
# cv.destroyAllWindows()


#Video
if not video.isOpened():
    print ("No camera found")
else:
    width = video.get(cv.CAP_PROP_FRAME_WIDTH)
    height = video.get(cv.CAP_PROP_FRAME_HEIGHT)
    print (f"{width} x {height}")
    while True:
        tf, frame = video.read()
        cv.imshow("Test", frame)

        if cv.waitKey(1) & 0xFF==ord('d'):
            # 0xFF is optional
            # 0xFF is literally 00000000 00000000 00000000 11111111, and used to filter the cv.waitkey() since in some os
            # the first 3 byte (1 byte = 8 bits) contains garbage and the only output used form cv.waitKey() is
            # the last byte only
            break

video.release()
cv.destroyAllWindows()