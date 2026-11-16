import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class FaceRecognitionNode(Node):
    def __init__(self):
        super().__init__('face_recognition_node')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('reference_dir', '')
        self.declare_parameter('person_name', 'Fabo')
        self.declare_parameter('greeting_text', 'Buongiorno Fabo, cosa facciamo oggi?')
        self.declare_parameter('greeting_cooldown_sec', 300.0)
        self.declare_parameter('detection_scale_factor', 1.1)
        self.declare_parameter('detection_min_neighbors', 5)
        self.declare_parameter('match_threshold', 0.72)
        self.declare_parameter('process_every_n_frames', 3)
        self.declare_parameter('publish_annotated_image', True)

        image_topic = self.get_parameter('image_topic').value
        self.reference_dir = self.get_parameter('reference_dir').value
        self.person_name = self.get_parameter('person_name').value
        self.greeting_text = self.get_parameter('greeting_text').value
        self.greeting_cooldown_sec = float(self.get_parameter('greeting_cooldown_sec').value)
        self.scale_factor = float(self.get_parameter('detection_scale_factor').value)
        self.min_neighbors = int(self.get_parameter('detection_min_neighbors').value)
        self.match_threshold = float(self.get_parameter('match_threshold').value)
        self.process_every_n_frames = max(1, int(self.get_parameter('process_every_n_frames').value))
        self.publish_annotated_image = bool(self.get_parameter('publish_annotated_image').value)

        cascade_path = os.path.join(cv2.data.haarcascades, 'haarcascade_frontalface_default.xml')
        self.face_cascade = cv2.CascadeClassifier(cascade_path)
        self.bridge = CvBridge()
        self.frame_count = 0
        self.last_greeting_time = 0.0
        self.reference_vectors = self._load_reference_vectors()

        self.tts_pub = self.create_publisher(String, '/tts_input', 10)
        self.annotated_pub = self.create_publisher(Image, '/face_recognition/image', 10)
        self.image_sub = self.create_subscription(Image, image_topic, self._image_callback, 10)

        self.get_logger().info(f'Face recognition listening on {image_topic}')
        if self.reference_vectors:
            self.get_logger().info(
                f'Loaded {len(self.reference_vectors)} reference face(s) for {self.person_name}'
            )
        else:
            self.get_logger().warn(
                'No reference faces loaded. Add photos to reference_dir or pass reference_dir:=...'
            )

    def _load_reference_vectors(self):
        if not self.reference_dir:
            self.get_logger().warn('Parameter reference_dir is empty')
            return []

        if not os.path.isdir(self.reference_dir):
            self.get_logger().warn(f'Reference directory not found: {self.reference_dir}')
            return []

        vectors = []
        valid_ext = ('.jpg', '.jpeg', '.png', '.bmp', '.webp')
        for filename in sorted(os.listdir(self.reference_dir)):
            if not filename.lower().endswith(valid_ext):
                continue

            path = os.path.join(self.reference_dir, filename)
            image = cv2.imread(path)
            if image is None:
                self.get_logger().warn(f'Cannot read reference image: {path}')
                continue

            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            faces = self._detect_faces(gray)
            if len(faces) == 0:
                self.get_logger().warn(f'No face found in reference image: {path}')
                continue

            face = max(faces, key=lambda rect: rect[2] * rect[3])
            vectors.append(self._face_vector(gray, face))

        return vectors

    def _detect_faces(self, gray):
        return self.face_cascade.detectMultiScale(
            gray,
            scaleFactor=self.scale_factor,
            minNeighbors=self.min_neighbors,
            minSize=(80, 80),
        )

    def _face_vector(self, gray, face_rect):
        x, y, w, h = face_rect
        face = gray[y:y + h, x:x + w]
        face = cv2.equalizeHist(face)
        face = cv2.resize(face, (96, 96), interpolation=cv2.INTER_AREA)
        vector = face.astype(np.float32).flatten()
        norm = np.linalg.norm(vector)
        if norm > 0:
            vector /= norm
        return vector

    def _best_match_score(self, vector):
        if not self.reference_vectors:
            return 0.0
        scores = [float(np.dot(vector, ref)) for ref in self.reference_vectors]
        return max(scores)

    def _say_greeting(self):
        now = time.monotonic()
        if now - self.last_greeting_time < self.greeting_cooldown_sec:
            return

        self.last_greeting_time = now
        msg = String()
        msg.data = self.greeting_text
        self.tts_pub.publish(msg)
        self.get_logger().info(self.greeting_text)

    def _image_callback(self, msg):
        self.frame_count += 1
        if self.frame_count % self.process_every_n_frames != 0:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Cannot convert image: {exc}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self._detect_faces(gray)

        recognized = False
        for face in faces:
            vector = self._face_vector(gray, face)
            score = self._best_match_score(vector)
            is_match = score >= self.match_threshold
            recognized = recognized or is_match

            if self.publish_annotated_image:
                x, y, w, h = face
                color = (0, 200, 0) if is_match else (0, 165, 255)
                label = f'{self.person_name} {score:.2f}' if is_match else f'unknown {score:.2f}'
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                cv2.putText(
                    frame,
                    label,
                    (x, max(20, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    color,
                    2,
                    cv2.LINE_AA,
                )

        if recognized:
            self._say_greeting()

        if self.publish_annotated_image:
            annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FaceRecognitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
