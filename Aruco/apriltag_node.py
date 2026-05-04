#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
import time
import serial
from pupil_apriltags import Detector

from geometry_msgs.msg import Twist
from std_msgs.msg import String

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
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
 
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
        # Alpha = EMA filter weight, 0<alpha<1 with the bigger the alpha value meaning the smooting is more
        # biased toward recent data (less smoothing)
        self.declare_parameter('alpha',             0.70) 

        self.declare_parameter('display_video', False)

        self.last_seen = 0.0
        self.x_f = 0.0
        self.z_f = 0.0
 
        # Load all params
        self.load_params()

        self.detector = Detector(families='tag36h11')

        # QoS
        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publisher 
        self.cmd_pub = self.create_publisher(
            Twist, 
            self.cmd_vel_topic, 
            qos_cmd
        )
        
        self.get_logger().info("April Tag Docking Initialized")

        # Video capture initialization
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            self.get_logger().info('Log: Camera Error')
            raise RuntimeError('Camera Error')

        self.timer = self.create_timer(self.send_period, self.april_tag_detection)
    
    def load_params(self):
        g = self.get_parameter

        self.camera_index     = g('camera_index').value
        self.target_tag_id    = g('target_tag_id').value
        self.tag_size         = g('tag_size').value
        self.cmd_vel_topic    = g('cmd_vel_topic').value
 
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

        self.display_video = g('display_video').value
        


    def april_tag_detection(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("No frame detected")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        result = self.detector.detect(
            gray, 
            estimate_tag_pose=True,
            camera_params=[self.fx, self.fy, self.cx, self.cy],
            tag_size=self.tag_size)

        target = next((t for t in result if t.tag_id == self.target_tag_id), None)

        now = time.time()
        x, z = 0.0, 0.0
        v, w = 0.0, 0.0

        if target is None:
            if now - self.last_seen > self.lost_tag_timeout:
                state = "SEARCH"
                v = 0.0
                w = self.search_w
            else:
                state = "WAIT"
                v = 0.0
                w = 0.0
        else:
            self.last_seen = now

            x_raw = float(target.pose_t[0][0])
            z_raw = float(target.pose_t[2][0])
            
            # Filter raw data by smoothing it between old and new data with alpha as the proportion
            self.x_f = self.alpha * x_raw + (1-self.alpha) * self.x_f
            self.z_f = self.alpha * z_raw + (1-self.alpha) * self.z_f

            # Right hand rule (x=middle, y= thumb, z=pointing finger) with z facing away and x towards the left
            x = self.x_f
            z = self.z_f

            if abs(x) < 0.02:
                x = 0.0

            if z > self.approach_z: # 1.25
                state = "FAST"
                v = self.k_v * (z - self.dock_z)
                w = -self.k_w * x

            elif z > self.slow_z: # 0.5
                state = "SLOW"
                v = 0.4
                w = -self.k_w* x

            elif z > self.dock_z: # 0.25
                state = "ALIGN"
                v = 0.2
                w = -1.2 * x

            else:
                state = "DOCKED"
                v = 0.0
                w = 0.0

            v = clamp(v, self.min_v, self.max_v)
            w = clamp(w, -self.max_w, self.max_w)

        # Publish command
        twist = Twist()
        twist.linear.x  = v
        twist.angular.z = w
        self.cmd_pub.publish(twist)

        self.get_logger().debug(f'[{state}] x = {x}, z = {z}, v = {v}, w = {w}')
        self.get_logger().info(f'[{state:<8}]  v={v:.3f}  w={w:.3f}', throttle_duration_sec=0.5)

        # Draw bbox
        if self.display_video:
            if target is not None:
                corners = target.corners.astype(int)
                for i in range(4):
                    pt1 = tuple(corners[i])
                    pt2 = tuple(corners[(i+1)%4])
                    cv2.line(frame, pt1, pt2, (0,255,0), 2)
                center = tuple(target.center.astype(int))
                cv2.circle(frame, center, 5, (0,0,255), -1)

            cv2.putText(frame, f'STATE={state}',
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f'v={v:.3f}  w={w:.3f}',
                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow('Live feed', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('User pressed Q – shutting down.')
            rclpy.shutdown()

    def destroy_node(self):
        self.get_logger().info('Stopping robot')
 
        # Publish a zero-velocity command before exiting
        stop = Twist()
        self.cmd_pub.publish(stop)
 
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()
 
 
# ── Entry point ────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = AprilTag()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    
if __name__ == '__main__':
    main()