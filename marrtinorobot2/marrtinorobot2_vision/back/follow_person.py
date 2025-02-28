import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class FollowPersonNode(Node):
    def __init__(self):
        super().__init__('follow_person_node')
        self.bridge = CvBridge()

        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Load pre-trained deep learning model for person detection
        self.net = cv2.dnn.readNet('path/to/person_detection_model.weights', 'path/to/person_detection_model.cfg')
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Perform person detection using deep learning
        detected_persons = self.detect_person(cv_image)

        # Implement your follow person logic here
        if detected_persons:
            self.follow_person(detected_persons[0])

    def detect_person(self, cv_image):
        # Resize image for the neural network
        blob = cv2.dnn.blobFromImage(cv_image, 0.007843, (300, 300), 127.5)

        # Set the input to the neural network
        self.net.setInput(blob)

        # Forward pass and get the detection
        detections = self.net.forward()

        detected_persons = []

        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > 0.5:  # Adjust confidence threshold as needed
                class_id = int(detections[0, 0, i, 1])
                if class_id == 15:  # Assuming class 15 corresponds to a person
                    box = detections[0, 0, i, 3:7] * np.array([cv_image.shape[1], cv_image.shape[0], cv_image.shape[1], cv_image.shape[0]])
                    detected_persons.append(box.astype(int))

        return detected_persons

    def follow_person(self, person_box):
        # Implement your follow person control logic here
        # For simplicity, we'll calculate the centroid of the bounding box
        centroid_x = (person_box[0] + person_box[2]) // 2
        centroid_y = (person_box[1] + person_box[3]) // 2

        self.get_logger().info(f"Following person at: ({centroid_x}, {centroid_y})")

def main(args=None):
    rclpy.init(args=args)
    node = FollowPersonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
