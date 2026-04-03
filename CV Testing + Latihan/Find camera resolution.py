import cv2

print("Scanning for cameras...")

for i in range(5):  # checks index 0 to 4
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        width  = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print(f"Camera found at index {i} → Resolution: {width} x {height}")
        cap.release()
    else:
        print(f"No camera at index {i}")