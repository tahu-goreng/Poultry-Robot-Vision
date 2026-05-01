import cv2 as cv
from pupil_apriltags import Detector

detector = Detector(families="tag36h11")

cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Camera index invalid")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("No frame detected")
    
    bw = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    result = detector.detect(bw)

    print(result)
    # print(result[0].corners[0][0])
    # print(type(result[0].corners[0][0]))

    for tag in result: 
        corners = tag.corners
        for j in range(4):
            pt1 = tuple(corners[j].astype(int))
            pt2 = tuple(corners[(j + 1) % 4].astype(int))
            cv.line(frame, (corners[j].astype(int)), (corners[(j+1)%4].astype(int)), (0, 0, 255), 2)

        planar_post = tag.pose_t
        rot_pose = tag.pose_R
        # print(f'{rot_pose} "/n" {planar_post}')
    

    cv.imshow("Camera 0", frame)
    
    if cv.waitKey(1) & 0xFF ==ord('q'):
        break


cv.destroyAllWindows()
cap.release()