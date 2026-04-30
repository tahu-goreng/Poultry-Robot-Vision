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
 
        self._load_params()

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
        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, qos_cmd)
        
        self.get_logger().info("April Tag Docking")

        self.timer = self.create_timer(timer berapa?????????,self.april_tag_detection)
    
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
        last_seen =
        cap = cv2.VideoCapture(self.camera_index)
        if not cap.isOpened():
            self.get_logger().info("No tags found")
            last_seen = time.time()
        else:

def main(args=None):
    rclpy.init(args=args) #initialized ros

    node = AprilTag()
    rclpy.spin(node)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()