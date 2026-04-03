import cv2 as cv
from cv2 import aruco

#Camera matrices
distortion = ???
    #to be written

marker_size = 10 #Base this on the real life unit

source = cv.VideoCapture(0)

while true:
    #Capture video
    condition, frame = source.read()


    #Analyze video

    #Exit function
    if cv.waitKey(0) == "q":
        cv.destroyAllWindows()
        break
