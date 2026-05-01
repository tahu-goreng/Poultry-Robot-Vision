#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
import time
import serial
from pupil_apriltags import Detector

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from pupil_apriltags import Detector

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Helper 
def clamp(v, vmin, vmax):
    return max(vmin, min(v, vmax))

# ROS2 node
class AprilTag(Node):
    def __init__(self):
        super().__init__("April_tag_node")
        self.declare_parameter('camera_index',      0)
        self.declare_parameter('target_tag_id',     0)
        self.declare_parameter('tag_size',          0.12)
 
        self.declare_parameter('fx', 640.0)
        self.declare_parameter('fy', 640.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
 
        self.declare_parameter('k_v',  0.55)
        self.declare_parameter('k_w',  1.80)
 
        self.declare_parameter('max_v', 0.30)
        self.declare_parameter('min_v', 0.00)
        self.declare_parameter('max_w', 1.00)
 
        self.declare_parameter('search_w',          0.35)
        self.declare_parameter('approach_z',        1.20)
        self.declare_parameter('slow_z',            0.50)
        self.declare_parameter('dock_z',            0.25)
        self.declare_parameter('center_tol',        0.03)
 
        self.declare_parameter('send_period',       0.05)   # seconds
        self.declare_parameter('lost_tag_timeout',  0.80)   # seconds
        self.declare_parameter('alpha',             0.70)   # EMA filter weight
 
        # Load all params
        self.load_params()

        self.detector = Detector(families='tag36h11')

        # QoS
        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
 
        # Publishers
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel_bridge', 10)
        self.state_pub = self.create_publisher(String, '/docking_state', 10)

        # Publisher 
        self.cmd_pub = self.create_publisher(Twist, self.('cmd_vel_topic').value, qos_cmd)
        
        self.get_logger().info("April Tag Docking")

        # Video capture initialization
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            self.get_logger().info('Log: Camera Error')
            raise RuntimeError('Camera Error')

        self.timer = self.create_timer(self.send_period, self.april_tag_detection)
    
    def load_params(self):
        g = self.get_parameters

        self.camera_index     = g('camera_index').value
        self.target_tag_id    = g('target_tag_id').value
        self.tag_size         = g('tag_size').value
 
        self.fx = g('fx').value
        self.fy = g('fy').value
        self.cx = g('cx').value
        self.cy = g('cy').value
 
        self.k_v  = g('k_v').value
        self.k_w  = g('k_w').value
 
        self.max_v = g('max_v').value
        self.min_v = g('min_v').value
        self.max_w = g('max_w').value
 
        self.search_w       = g('search_w').value
        self.approach_z     = g('approach_z').value
        self.slow_z         = g('slow_z').value
        self.dock_z         = g('dock_z').value
        self.center_tol     = g('center_tol').value
 
        self.send_period        = g('send_period').value
        self.lost_tag_timeout   = g('lost_tag_timeout').value
        self.alpha              = g('alpha').value

        


    def april_tag_detection (self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("No frame detected")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        result = self.detector.detect(
            gray, 
            estimate_tag_pose=True,
            camera_params=[self.fx, self.fy, self.cx, self.cy],
            tag_size=self.tag_size)

    # Display image

        target = []
        now = 0
        v = 0.0
        w = 0.0

        for tag in result:
            if tag.tag_id == self.target_tag_id:
                target = tag
                break
        
        if target is None:
            if now - last_seen > LOST_TAG_TIMEOUT:
                state = "SEARCH"
                v = 0.0
                w = self.search_w
            else:
                state = "WAIT"
                v = 0.0
                w = 0.0
        else:
            last_seen = now
            # Draw bbox
            corners = target.corners.astype(int)
            for i in range(4):
                pt1 = tuple(corners[i])
                pt2 = tuple(corners[(i+1)%4])
                cv2.line(frame, pt1, pt2, (0,255,0), 2)

            center = tuple(target.center.astype(int))
            cv2.circle(frame, center, 5, (0,0,255), -1)

            x_raw = float(target.pose_t[0][0])
            z_raw = float(target.pose_t[2][0])
            
            x_f = self.alpha * x_raw + (1-self.alpha) * x_f
            z_f = self.alpha * z_raw + (1-self.alpha) * z_f

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
            if z > self.approach_z:
                state = "FAST"
                v = self.k_v * (z - self.dock_z)
                w = -self.k_w * x

            elif z > self.slow_z:
                state = "SLOW"
                v = 0.4
                w = -self.k_w* x

            elif z > self.dock_z:
                state = "ALIGN"
                v = 0.2
                w = -1.2 * x

            else:
                state = "DOCKED"
                v = 0.0
                w = 0.0

            
            cv2.imshow('Robot vision', frame)


def main(args=None):
    rclpy.init(args=args) #initialized ros

    node = AprilTag()
    rclpy.spin(node)
    
    rclpy.shutdown()
    cv2.destroyAllWindows()
    cap.release()
    
if __name__ == '__main__':
    main()