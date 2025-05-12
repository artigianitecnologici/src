#!/usr/bin/env python3
# Copyright 2025 robotics-3d.com

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class PIDController:
    def __init__(self, kp, ki, kd, integral_limit=1000):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        self.integral_limit = integral_limit

    def compute(self, error):
        self.integral += error
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)  # anti-windup
        derivative = error - self.prev_error
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


class FaceRecognitionAndTrackingNode(Node):
    def __init__(self):
        super().__init__('face_recognition_and_tracking_node')

        self.net = cv2.dnn.readNetFromCaffe(
            'deploy.prototxt',
            'res10_300x300_ssd_iter_140000.caffemodel'
        )

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )
        self.image_face_detection = self.create_publisher(Image, '/face_detector/image_raw', 10)

        self.dynamixel_control = self.create_publisher(Float64, '/pan_controller/command', 10)
        self.dynamixel_control_tilt = self.create_publisher(Float64, '/tilt_controller/command', 10)
        self.face_count_pub = self.create_publisher(Float64, '/nroface', 10)

        self.servomaxx = 1023
        self.servomaxy = 1023
        self.servomin = 0
        self.center_pos_x = 512
        self.center_pos_y = 512
        self.current_pos_x = float(self.center_pos_x)
        self.current_pos_y = float(self.center_pos_y)

        self.pid_x = PIDController(0.05, 0.001, 0.01)
        self.pid_y = PIDController(0.05, 0.001, 0.01)

        self.screenmaxx = 640
        self.screenmaxy = 480

        # Posizione iniziale
        self.dynamixel_control.publish(Float64(data=self.position_to_degrees(self.center_pos_x)))
        self.dynamixel_control_tilt.publish(Float64(data=self.position_to_degrees(self.center_pos_y - 100)))

        self.get_logger().info("Face Tracker Controller v.1.1 - Ottimizzato")

    def position_to_degrees(self, position):
        return position * 300 / 1023

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            frame = cv2.flip(frame, 1)
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0))
        self.net.setInput(blob)
        detections = self.net.forward()

        face_count = 0
        best_confidence = 0
        best_box = None

        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > 0.5 and confidence > best_confidence:
                best_confidence = confidence
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                best_box = box.astype("int")
                face_count += 1

        if best_box is not None:
            (startX, startY, endX, endY) = best_box
            cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 0, 255), 2)
            face_center_x = (startX + endX) // 2
            face_center_y = (startY + endY) // 2
            offset_x = face_center_x - (w // 2)
            offset_y = face_center_y - (h // 2)
            self.track_face(offset_x, offset_y)

        face_count_msg = Float64()
        face_count_msg.data = float(face_count)
        self.face_count_pub.publish(face_count_msg)

        try:
            ros_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_face_detection.publish(ros_image_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish image: {e}')

    def track_face(self, x, y):
        control_x = self.pid_x.compute(x)
        control_y = self.pid_y.compute(y)

        # Asse X (pan)
        new_pos_x = self.current_pos_x - control_x
        if self.servomin <= new_pos_x <= self.servomaxx:
            if abs(new_pos_x - self.current_pos_x) > 1.0:
                self.current_pos_x = new_pos_x
                self.dynamixel_control.publish(Float64(data=self.position_to_degrees(512 - self.current_pos_x)))

        # Asse Y (tilt)
        new_pos_y = self.current_pos_y + control_y
        if self.servomin <= new_pos_y <= self.servomaxy:
            if abs(new_pos_y - self.current_pos_y) > 1.0:
                self.current_pos_y = new_pos_y
                self.dynamixel_control_tilt.publish(Float64(data=self.position_to_degrees(512 - self.current_pos_y)))


def main(args=None):
    rclpy.init(args=args)
    node = FaceRecognitionAndTrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
