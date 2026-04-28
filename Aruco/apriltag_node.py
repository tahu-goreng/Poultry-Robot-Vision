import rclpy
from rclpy.node import node

import cv2
import time
import serial
from pupil_apriltags import detector

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class AprilTag(Node):

    def __init__(self):
        super().__init__('aprilTag_detection')

        # ===================== PARAMETERS =====================
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.declare_parameter('front_threshold', 0.6)   # meter
        self.declare_parameter('side_threshold',  0.7)   # meter

        self.declare_parameter('forward_speed', 0.30)    # m/s
        self.declare_parameter('turn_speed',    0.80)    # rad/s

        # ===================== QoS (IMPORTANT FIX) =====================
        qos_laser = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ===================== SUBSCRIBER =====================
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self.scan_callback,
            qos_laser        # <<< FIXED QoS
        )

        # ===================== PUBLISHER =====================
        self.cmd_pub = self.create_publisher(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            qos_cmd
        )

        self.get_logger().info("Reactive Obstacle Avoidance STARTED (QoS FIXED)")


    # ========================================================
    # LaserScan callback
    # ========================================================
    def scan_callback(self, msg: LaserScan):

        ranges = list(msg.ranges)
        size   = len(ranges)

        # Replace invalid readings
        ranges = [
            r if (msg.range_min < r < msg.range_max) else float('inf')
            for r in ranges
        ]

        # ===================== ZONES =====================
        front_idx = size // 2
        front = min(ranges[front_idx - 10 : front_idx + 10])

        left  = min(ranges[int(size * 0.70) : int(size * 0.80)])
        right = min(ranges[int(size * 0.20) : int(size * 0.30)])

        front_th = self.get_parameter('front_threshold').value
        side_th  = self.get_parameter('side_threshold').value

        fwd_speed  = self.get_parameter('forward_speed').value
        turn_speed = self.get_parameter('turn_speed').value

        cmd = Twist()

        # ===================== DECISION LOGIC =====================
        if front > front_th:
            # Path is clear → go forward
            cmd.linear.x  = fwd_speed
            cmd.angular.z = 0.0

        else:
            # Obstacle ahead → turn to freer side
            cmd.linear.x = 0.0

            if left > right and left > side_th:
                cmd.angular.z =  turn_speed     # turn left
            else:
                cmd.angular.z = -turn_speed     # turn right

        self.cmd_pub.publish(cmd)


# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = ReactiveAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()