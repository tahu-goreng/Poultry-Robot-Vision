import cv2
import time
import serial
from pupil_apriltags import Detector

# =========================
# CONFIG
# =========================
SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 115200
CAMERA_INDEX = 0
TARGET_TAG_ID = 0
TAG_SIZE = 0.12

FX = 640.0
FY = 640.0
CX = 320.0
CY = 240.0

K_V = 0.55
K_W = 1.80

MAX_V = 0.30
MIN_V = 0.00
MAX_W = 1.00

SEARCH_W = 0.35
APPROACH_Z = 1.20
SLOW_Z = 0.50
DOCK_Z = 0.25
CENTER_TOL = 0.03

SEND_PERIOD = 0.05
LOST_TAG_TIMEOUT = 0.8

# FILTER (biar smooth)
ALPHA = 0.7
x_f, z_f = 0.0, 0.0

# =========================
# HELPER
# =========================
def clamp(v, vmin, vmax):
    return max(vmin, min(v, vmax))

def draw_text(img, text, y, color=(0,255,0)):
    cv2.putText(img, text, (10,y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                color, 2)

# =========================
# SERIAL
# =========================
ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.05)
time.sleep(2)

def send_cmd(v, w):
    msg = f"CMD {v:.3f} {w:.3f}\n"
    ser.write(msg.encode())

def send_stop():
    send_cmd(0.0, 0.0)

# ARM robot
ser.write(b"ARM\n")
time.sleep(0.5)
send_stop()

# =========================
# APRILTAG
# =========================
detector = Detector(families="tag36h11")

# =========================
# CAMERA
# =========================
cap = cv2.VideoCapture(CAMERA_INDEX)
if not cap.isOpened():
    raise RuntimeError("Camera error")

last_seen = time.time()
last_send = time.time()

print("START DOCKING...")

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    tags = detector.detect(
        gray,
        estimate_tag_pose=True,
        camera_params=[FX, FY, CX, CY],
        tag_size=TAG_SIZE
    )

    target = None
    for tag in tags:
        if tag.tag_id == TARGET_TAG_ID:
            target = tag
            break

    now = time.time()
    v, w = 0.0, 0.0

    if target is None:
        if now - last_seen > LOST_TAG_TIMEOUT:
            state = "SEARCH"
            v = 0.0
            w = SEARCH_W
        else:
            state = "WAIT"
            v = 0.0
            w = 0.0

    else:
        last_seen = now

        # =========================
        # DRAW BOUNDING BOX
        # =========================
        corners = target.corners.astype(int)
        for i in range(4):
            pt1 = tuple(corners[i])
            pt2 = tuple(corners[(i+1)%4])
            cv2.line(frame, pt1, pt2, (0,255,0), 2)

        center = tuple(target.center.astype(int))
        cv2.circle(frame, center, 5, (0,0,255), -1)

        # =========================
        # RAW DATA
        # =========================
        x_raw = float(target.pose_t[0][0])
        z_raw = float(target.pose_t[2][0])

        # =========================
        # FILTER (SMOOTH)
        # =========================
        x_f = ALPHA * x_raw + (1-ALPHA) * x_f
        z_f = ALPHA * z_raw + (1-ALPHA) * z_f

        x = x_f
        z = z_f

        # =========================
        # DEADZONE
        # =========================
        if abs(x) < 0.02:
            x = 0.0

        # =========================
        # STATE MACHINE
        # =========================
        if z > APPROACH_Z:
            state = "FAST"
            v = K_V * (z - DOCK_Z)
            w = -K_W * x

        elif z > SLOW_Z:
            state = "SLOW"
            v = 0.15
            w = -K_W * x

        elif z > DOCK_Z:
            state = "ALIGN"
            v = 0.05
            w = -1.2 * x

        else:
            state = "DOCKED"
            v = 0.0
            w = 0.0

        draw_text(frame, f"x={x:.2f}", 30)
        draw_text(frame, f"z={z:.2f}", 60)

    v = clamp(v, MIN_V, MAX_V)
    w = clamp(w, -MAX_W, MAX_W)

    draw_text(frame, f"STATE={state}", 90)
    draw_text(frame, f"v={v:.2f}", 120)
    draw_text(frame, f"w={w:.2f}", 150)

    # =========================
    # SEND COMMAND
    # =========================
    if now - last_send > SEND_PERIOD:
        send_cmd(v, w)
        last_send = now

    cv2.imshow("Docking", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# STOP
send_stop()
ser.write(b"DISARM\n")

cap.release()
cv2.destroyAllWindows()
ser.close()