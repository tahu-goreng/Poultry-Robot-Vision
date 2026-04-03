import cv2 as cv

video = cv.VideoCapture(1, cv.CAP_DSHOW)

if not video.isOpened():
    print("Camera not found!")
else:
    print("Camera opened! Press 'd' to quit.")

    while True:
        ret, frame = video.read()

        if not ret:
            print("Failed to grab frame")
            break

        cv.imshow("USB Camera", frame)
        key = cv.waitKey(1) & 0xFF  # 1ms delay, keeps window alive

        if key == ord('d'):
            break

video.release()
cv.destroyAllWindows()