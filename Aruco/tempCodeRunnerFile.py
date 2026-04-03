dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_50)
board = cv.aruco.CharucoBoard((7, 5), 0.04, 0.03, dictionary)

detector_params = cv.aruco.DetectorParameters()
charuco_params  = cv.aruco.CharucoParameters()
charuco_detector = cv.aruco.CharucoDetector(board, charuco_params, detector_params)

cap = cv.VideoCapture(camera_index)
saved_frames = []

print(f"Capturing calibration frames. Target: {num_frames}")
print("Press SPACE to capture, Q to finish.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    charuco_corners, charuco_ids, marker_corners, marker_ids = charuco_detector.detectBoard(gray)

    display = frame.copy()
    if marker_ids is not None:
        cv.aruco.drawDetectedMarkers(display, marker_corners, marker_ids)
    if charuco_corners is not None and len(charuco_corners) >= 6:
        cv.aruco.drawDetectedCornersCharuco(display, charuco_corners, charuco_ids)
        count_text = f"Corners: {len(charuco_corners)} | Saved: {len(saved_frames)}/{num_frames}"
        cv.putText(display, count_text, (10, 30),
                    cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    cv.imshow("Calibration Capture", display)
    key = cv.waitKey(1) & 0xFF

    if key == ord(' ') and charuco_corners is not None and len(charuco_corners) >= 6:
        saved_frames.append(frame.copy())
        print(f"  Saved frame {len(saved_frames)}/{num_frames}")
    elif key == ord('q') or len(saved_frames) >= num_frames:
        break

cap.release()
cv.destroyAllWindows()
return saved_frames, gray.shape[::-1]  # frames + (width, height)