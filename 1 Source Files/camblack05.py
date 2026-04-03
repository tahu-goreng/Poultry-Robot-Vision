# -*- coding: utf-8 -*-
import cv2
import numpy as np
import time
import serial

# ===============================
# DOCK PIXEL (HASIL KALIBRASI)
# ===============================
DOCK_X = 291
DOCK_Y = 353

DOCK_TOL_X = 5     # pixel
DOCK_TOL_Y = 6     # pixel

# ===============================
# ESP SERIAL
# ===============================
ESP_PORT = "/dev/ttyACM0"
ESP_BAUD = 115200

# ===============================
# CONTROL (ROBOT BERAT)
# ===============================
K_W = 0.0045
K_V = 0.0040

MAX_W = 0.8
MAX_V = 0.6
MIN_V = 0.4    # ⬅️ WAJIB UNTUK TORSI

# ===============================
# HSV BLACK
# ===============================
lower_black = np.array([0, 0, 0])
upper_black = np.array([180, 255, 50])

# ===============================
# INIT
# ===============================
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Camera error")
    exit()

ser = serial.Serial(ESP_PORT, ESP_BAUD, timeout=0.1)
time.sleep(0.5)

def esp_send(cmd):
    ser.write((cmd + "\n").encode())

def esp_cmd(v, w):
    esp_send(f"CMD {v:.3f} {w:.3f}")

def clip(x, lo, hi):
    return max(lo, min(x, hi))

kernel = np.ones((5,5), np.uint8)
armed = False

print("=== PIXEL DOCKING STARTED ===")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_black, upper_black)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        found = False

        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 800:
                x,y,w,h = cv2.boundingRect(c)
                cx = x + w//2
                cy = y + h//2
                found = True

                err_x = cx - DOCK_X
                err_y = DOCK_Y - cy

                cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
                cv2.circle(frame,(cx,cy),5,(0,0,255),-1)
                cv2.putText(frame,f"Center ({cx},{cy})",(x,y-10),
                            cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),2)

        if not armed:
            esp_send("ARM")
            time.sleep(0.1)
            armed = True

        v = 0.0
        w = 0.0

        if found:
            w = clip(-K_W * err_x, -MAX_W, MAX_W)

            if abs(err_x) < 20:
                v = clip(K_V * err_y, MIN_V, MAX_V)

            # === DOCK CONDITION (PIXEL ONLY)
            if abs(err_x) < DOCK_TOL_X and abs(err_y) < DOCK_TOL_Y:
                esp_cmd(0,0)
                time.sleep(0.2)
                esp_send("DISARM")
                print(">>> DOCKED EXACT PIXEL <<<")
                break
        else:
            w = 0.6  # search spin

        esp_cmd(v,w)

        cv2.imshow("Original", frame)
        cv2.imshow("Mask", mask)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    esp_cmd(0,0)
    esp_send("DISARM")
    cap.release()
    ser.close()
    cv2.destroyAllWindows()

print("Exit")
