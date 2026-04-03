import cv2 as cv
import numpy as np


board = cv.aruco.CharucoBoard((squareX, squareY), squareLength, markerLength, dictionary)
detector = cv.aruco.CharucoDetector(board, params, detectorParams)

input_vid = cv.VideoCapture("C:\Users\User\Documents\大學\Project\Poultry Robot Vision\IMG_E2872.MOV")

all_charuco_corners = []
all_charuco_ids = []
all_image_points = []
all_object_points = []
all_images = []
image_size = None

while input_vid.grab():
    ret, frame = input_vid.retrieve()
    if not ret :
        break
    
    markerId = []
    markerCorners = []
    current_charuco_markers = []
    currentIds = []
    current_object_points = []
    current_image_points = []

    detector.detectBoard(frame, current_charuco_markers)