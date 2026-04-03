#!/usr/bin/env python3

import os
import sys
import argparse
import time

import cv2
import numpy as np
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D
from geometry_msgs.msg import Pose2D


class YoloDetectorNode(Node):
    def __init__(self):
        #Node name = yolo_detector
        super().__init__('yolo_detector')

        #Set parameters
        self.declare_parameter('model',      '/home/waesco704/Documents/my_model/my_model.pt')
        self.declare_parameter('source',     'usb0')
        self.declare_parameter('thresh',     0.5)
        self.declare_parameter('record',     False)

        model_path  = self.get_parameter('model').value
        img_source  = self.get_parameter('source').value
        self.min_thresh = self.get_parameter('thresh').value
        record      = self.get_parameter('record').value

        if not os.path.exists(model_path):
            self.get_logger().error('Model not found: ' + model_path)
            sys.exit(1)

        self.model  = YOLO(model_path, task='detect')
        self.labels = self.model.names

        if 'usb' in img_source:
            self.source_type = 'usb'
            usb_idx = int(img_source[3:])
        else:
            self.get_logger().error(f'Invalid source: {img_source}')
            sys.exit(1)

        self.cap = cv2.VideoCapture(usb_idx)
        if not self.cap.isOpened():
            self.get_logger().error('Cannot open camera.')
            sys.exit(1)

        # --- Publishers ---
        # Option A: lightweight string  →  "xmin ymin xmax ymax label conf"
        self.pub_raw = self.create_publisher(String, 'yolo/detections_raw', 10)

        # Option B: standard vision_msgs (recommended for downstream nodes)
        self.pub_det = self.create_publisher(Detection2DArray, 'yolo/detections', 10)

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

        self.bbox_colors = [
            (164,120,87),(68,148,228),(93,97,209),(178,182,133),(88,159,106),
            (96,202,231),(159,124,168),(169,162,241),(98,118,150),(172,176,184)
        ]

        self.frame_rate_buffer = []
        self.avg_frame_rate    = 0.0
        self.get_logger().info('YoloDetectorNode started.')


    def timer_callback(self):
        t_start = time.perf_counter()

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn('Frame read failed.')
            return

        results    = self.model(frame, verbose=False)
        detections = results[0].boxes

        # Prepare Detection2DArray message
        det_array_msg           = Detection2DArray()
        det_array_msg.header.stamp    = self.get_clock().now().to_msg()
        det_array_msg.header.frame_id = 'camera'

        object_count = 0

        for i in range(len(detections)):
            # --- Bounding box ---
            xyxy = detections[i].xyxy.cpu().numpy().squeeze()
            xmin, ymin, xmax, ymax = xyxy.astype(int)

            classidx  = int(detections[i].cls.item())
            classname = self.labels[classidx]
            conf      = detections[i].conf.item()

            if conf < self.min_thresh:
                continue

            object_count += 1

            # ── Publish raw string (replaces your print statement) ──────
            raw_msg      = String()
            raw_msg.data = f'{xmin} {ymin} {xmax} {ymax} {classname} {conf:.2f}'
            self.pub_raw.publish(raw_msg)
            self.get_logger().debug(raw_msg.data)

            # ── Build Detection2D message ────────────────────────────────
            det_msg = Detection2D()
            det_msg.header = det_array_msg.header

            bbox         = BoundingBox2D()
            cx           = (xmin + xmax) / 2.0
            cy           = (ymin + ymax) / 2.0
            bbox.center  = Pose2D(x=cx, y=cy)
            bbox.size_x  = float(xmax - xmin)
            bbox.size_y  = float(ymax - ymin)
            det_msg.bbox = bbox

            # Store class + confidence in results (vision_msgs convention)
            from vision_msgs.msg import ObjectHypothesisWithPose, ObjectHypothesis
            hyp            = ObjectHypothesisWithPose()
            hyp.hypothesis = ObjectHypothesis(class_id=classname, score=conf)
            det_msg.results.append(hyp)

            det_array_msg.detections.append(det_msg)

            # --- Draw on frame ---
            color = self.bbox_colors[classidx % 10]
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)
            label     = f'{classname}: {int(conf*100)}%'
            lSize, bL = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            ly        = max(ymin, lSize[1] + 10)
            cv2.rectangle(frame, (xmin, ly-lSize[1]-10),
                          (xmin+lSize[0], ly+bL-10), color, cv2.FILLED)
            cv2.putText(frame, label, (xmin, ly-7),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1)

        # Publish Detection2DArray once per frame
        self.pub_det.publish(det_array_msg)

        # --- FPS overlay ---
        t_stop = time.perf_counter()
        fps    = 1.0 / max(t_stop - t_start, 1e-9)
        self.frame_rate_buffer.append(fps)
        if len(self.frame_rate_buffer) > 200:
            self.frame_rate_buffer.pop(0)
        self.avg_frame_rate = float(np.mean(self.frame_rate_buffer))

        cv2.putText(frame, f'FPS: {self.avg_frame_rate:.2f}',
                    (10, 20), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2)
        cv2.putText(frame, f'Objects: {object_count}',
                    (10, 40), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2)
        cv2.imshow('YOLO detection results', frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info('Quit key pressed.')
            self.destroy_node()
            rclpy.shutdown()

    # ------------------------------------------------------------------
    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


# ======================================================================
def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()