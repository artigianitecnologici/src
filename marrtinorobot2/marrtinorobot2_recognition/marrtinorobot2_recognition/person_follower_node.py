#!/usr/bin/env python3
"""Low-duty-cycle person search and approach controller.

A visible face is used as a lightweight person detector. Inference is disabled
while idle and limited to a configurable rate while searching/following.
"""

import json
import math
import os
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory


def ros_image_to_bgr(msg: Image) -> np.ndarray:
    channels = 3
    row_bytes = int(msg.step) if msg.step else int(msg.width) * channels
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape((int(msg.height), row_bytes))
    image = raw[:, : int(msg.width) * channels].reshape((int(msg.height), int(msg.width), channels))
    encoding = (msg.encoding or '').lower()
    if encoding == 'rgb8':
        return image[:, :, ::-1]
    if encoding == 'bgr8':
        return image
    raise ValueError(f"encoding non supportato: {msg.encoding}")


class PersonFollowerNode(Node):
    IDLE = 'idle'
    SEARCHING = 'searching'
    FOUND = 'found'
    APPROACHING = 'approaching'

    def __init__(self):
        super().__init__('person_follower')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('scan_topic', '/scan_unfiltered')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('command_topic', '/person_follower/command')
        self.declare_parameter('status_topic', '/person_follower/status')
        self.declare_parameter('detection_topic', '/person_follower/detection')
        self.declare_parameter('speech_topic', '/speech/to_speak')
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence', 0.65)
        self.declare_parameter('inference_hz', 1.5)
        self.declare_parameter('search_speed', 0.16)
        self.declare_parameter('search_timeout_sec', 35.0)
        self.declare_parameter('confirm_frames', 2)
        self.declare_parameter('max_linear_speed', 0.12)
        self.declare_parameter('max_angular_speed', 0.22)
        self.declare_parameter('stop_distance', 0.70)
        self.declare_parameter('target_face_ratio', 0.24)
        self.declare_parameter('approach_timeout_sec', 30.0)
        self.declare_parameter('target_lost_sec', 2.5)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.command_topic = str(self.get_parameter('command_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.speech_topic = str(self.get_parameter('speech_topic').value)
        self.confidence = float(self.get_parameter('confidence').value)
        self.inference_period = 1.0 / max(0.2, float(self.get_parameter('inference_hz').value))
        self.search_speed = abs(float(self.get_parameter('search_speed').value))
        self.search_timeout = max(5.0, float(self.get_parameter('search_timeout_sec').value))
        self.confirm_frames = max(1, int(self.get_parameter('confirm_frames').value))
        self.max_linear = abs(float(self.get_parameter('max_linear_speed').value))
        self.max_angular = abs(float(self.get_parameter('max_angular_speed').value))
        self.stop_distance = max(0.35, float(self.get_parameter('stop_distance').value))
        self.target_face_ratio = max(0.08, float(self.get_parameter('target_face_ratio').value))
        self.approach_timeout = max(5.0, float(self.get_parameter('approach_timeout_sec').value))
        self.target_lost_sec = max(0.5, float(self.get_parameter('target_lost_sec').value))

        model_path = str(self.get_parameter('model_path').value or '').strip()
        if not model_path:
            model_path = os.path.join(
                get_package_share_directory('marrtinorobot2_recognition'),
                'models',
                'face_detection_yunet_2026may.onnx',
            )
        self.detector = cv2.FaceDetectorYN_create(
            model_path, '', (320, 240), self.confidence, 0.3, 5000
        )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sensor_qos = sensor_qos
        self.image_sub = None
        self.scan_sub = None
        self.command_sub = self.create_subscription(
            String, self.command_topic, self._on_command, 10
        )
        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter('cmd_vel_topic').value), 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.detection_pub = self.create_publisher(
            String, str(self.get_parameter('detection_topic').value), 10
        )
        self.speech_pub = self.create_publisher(String, self.speech_topic, 10)

        self.state = self.IDLE
        self.latest_image = None
        self.latest_scan = None
        self.last_scan_received = 0.0
        self.last_inference = 0.0
        self.action_started = 0.0
        self.last_seen = 0.0
        self.last_target = None
        self.detection_streak = 0
        self.search_direction = 1.0
        self.timer = self.create_timer(0.1, self._tick)
        self._publish_status('ready', 'Rilevatore persone pronto e inattivo.')
        self.get_logger().info(
            f"Person follower pronto: image={self.image_topic}, scan={self.scan_topic}, "
            f"inference={1.0 / self.inference_period:.1f} Hz (solo su comando)"
        )

    def _on_image(self, msg: Image):
        # Keep one ROS message only. No conversion/inference occurs while idle.
        self.latest_image = msg

    def _enable_camera(self):
        if self.image_sub is None:
            self.latest_image = None
            self.image_sub = self.create_subscription(
                Image, self.image_topic, self._on_image, self.sensor_qos
            )
        if self.scan_sub is None:
            self.latest_scan = None
            self.last_scan_received = 0.0
            self.scan_sub = self.create_subscription(
                LaserScan, self.scan_topic, self._on_scan, self.sensor_qos
            )

    def _disable_camera(self):
        if self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
        if self.scan_sub is not None:
            self.destroy_subscription(self.scan_sub)
            self.scan_sub = None
        self.latest_image = None
        self.latest_scan = None
        self.last_scan_received = 0.0

    def _on_scan(self, msg: LaserScan):
        self.latest_scan = msg
        self.last_scan_received = time.monotonic()

    def _on_command(self, msg: String):
        raw = (msg.data or '').strip()
        try:
            payload = json.loads(raw)
            action = str(payload.get('action', '')).lower().strip()
            direction = str(payload.get('direction', '')).lower().strip()
        except (json.JSONDecodeError, AttributeError):
            action = raw.lower()
            direction = ''

        if action in ('detect', 'search', 'find', 'cerca', 'rileva'):
            self._start_search(direction)
        elif action in ('approach', 'follow', 'come', 'avvicinati', 'vieni'):
            self._start_approach()
        elif action in ('stop', 'cancel', 'ferma'):
            self._finish('stopped', 'Ricerca persona fermata.', speak=False)
        elif action == 'status':
            self._publish_status(self.state, f'Stato rilevatore: {self.state}.')
        else:
            self._publish_status('error', f'Comando sconosciuto: {action}')

    def _start_search(self, direction=''):
        self._stop_robot()
        self._enable_camera()
        self.state = self.SEARCHING
        self.action_started = time.monotonic()
        self.last_inference = 0.0
        self.detection_streak = 0
        self.last_target = None
        self.search_direction = -1.0 if direction in ('left', 'sinistra') else 1.0
        self._publish_status(self.state, 'Cerco una persona ruotando lentamente.')

    def _start_approach(self):
        now = time.monotonic()
        self._enable_camera()
        if not self.last_target or now - self.last_seen > 5.0:
            self.state = self.APPROACHING
            self.action_started = now
            self.last_inference = 0.0
            self.detection_streak = 0
            self._publish_status(self.state, 'Cerco una persona e poi mi avvicino.')
            return
        self.state = self.APPROACHING
        self.action_started = now
        self._publish_status(self.state, 'Mi avvicino alla persona trovata.')

    def _detect_face(self):
        if self.latest_image is None:
            return None
        try:
            image = ros_image_to_bgr(self.latest_image)
            height, width = image.shape[:2]
            scale = min(1.0, 320.0 / max(width, height))
            small = cv2.resize(image, None, fx=scale, fy=scale) if scale < 1.0 else image
            small_height, small_width = small.shape[:2]
            self.detector.setInputSize((small_width, small_height))
            _, detections = self.detector.detect(small)
            best = None
            for face in detections if detections is not None else []:
                score = float(face[-1])
                if score < self.confidence:
                    continue
                x1, y1, face_width, face_height = [float(value) for value in face[:4]]
                x2 = x1 + face_width
                y2 = y1 + face_height
                x1 /= small_width
                x2 /= small_width
                y1 /= small_height
                y2 /= small_height
                x1, x2 = sorted((float(x1), float(x2)))
                y1, y2 = sorted((float(y1), float(y2)))
                area = max(0.0, x2 - x1) * max(0.0, y2 - y1)
                if best is None or area > best['area']:
                    best = {
                        'confidence': score,
                        'center_x': max(0.0, min(1.0, (x1 + x2) / 2.0)),
                        'face_ratio': max(0.0, min(1.0, y2 - y1)),
                        'area': area,
                    }
            return best
        except Exception as exc:
            self.get_logger().warning(f'Frame non analizzabile: {exc}')
            return None

    def _front_distance(self):
        scan = self.latest_scan
        if scan is None or not scan.ranges or not scan.angle_increment:
            return math.inf
        values = []
        for index, value in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            if abs(angle) <= math.radians(22) and math.isfinite(value) and value > scan.range_min:
                values.append(float(value))
        return min(values) if values else math.inf

    def _tick(self):
        if self.state == self.IDLE or self.state == self.FOUND:
            return
        now = time.monotonic()
        timeout = self.approach_timeout if self.state == self.APPROACHING else self.search_timeout
        if now - self.action_started > timeout:
            self._finish('timeout', 'Non ho trovato una persona.', speak=True)
            return

        if now - self.last_inference >= self.inference_period:
            self.last_inference = now
            target = self._detect_face()
            if target:
                self.last_target = target
                self.last_seen = now
                self.detection_streak += 1
                self.detection_pub.publish(String(data=json.dumps(target)))
            else:
                self.detection_streak = 0

        if self.state == self.SEARCHING:
            if self.detection_streak >= self.confirm_frames:
                self.state = self.FOUND
                self._stop_robot()
                self._disable_camera()
                self._publish_status(self.FOUND, 'Persona trovata.', target=self.last_target)
                self._speak('Persona trovata.')
                return
            # Reverse once so the search covers both sides without spinning fast.
            elapsed = now - self.action_started
            direction = self.search_direction if elapsed < self.search_timeout / 2.0 else -self.search_direction
            self._drive(0.0, direction * self.search_speed)
            return

        if self.last_target is None or now - self.last_seen > self.target_lost_sec:
            self._drive(0.0, self.search_direction * self.search_speed * 0.75)
            return

        scan_is_fresh = self.latest_scan is not None and now - self.last_scan_received <= 1.0
        distance = self._front_distance()
        if distance <= self.stop_distance:
            self._finish('arrived', 'Sono arrivato vicino alla persona.', speak=True)
            return
        target = self.last_target
        face_ratio = target['face_ratio']
        if face_ratio >= self.target_face_ratio:
            self._finish('arrived', 'Sono arrivato vicino alla persona.', speak=True)
            return
        error_x = target['center_x'] - 0.5
        angular = max(-self.max_angular, min(self.max_angular, -error_x * 0.7))
        aligned = abs(error_x) < 0.28
        ratio_error = max(0.0, self.target_face_ratio - face_ratio)
        # Never advance without a recent laser scan. Angular centering remains
        # allowed so the target stays in view while waiting for the sensor.
        linear = (
            min(self.max_linear, 0.03 + ratio_error * 0.55)
            if aligned and scan_is_fresh
            else 0.0
        )
        self._drive(linear, angular)

    def _drive(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def _stop_robot(self):
        self.cmd_pub.publish(Twist())

    def _speak(self, message):
        self.speech_pub.publish(String(data=message))

    def _publish_status(self, state, message, **extra):
        payload = {'state': state, 'message': message, 'timestamp': time.time()}
        payload.update(extra)
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.get_logger().info(message)

    def _finish(self, state, message, speak=False):
        self._stop_robot()
        self._disable_camera()
        self.state = self.IDLE
        self._publish_status(state, message)
        if speak:
            self._speak(message)

    def destroy_node(self):
        try:
            if rclpy.ok(context=self.context):
                self._stop_robot()
        except Exception:
            # Launch may invalidate the ROS context before invoking cleanup.
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowerNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
