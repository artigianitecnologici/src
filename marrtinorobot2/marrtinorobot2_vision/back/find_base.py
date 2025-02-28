import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import apriltag
import cv2
import numpy as np

class TagRecognizerNode(Node):
    def __init__(self):
        super().__init__('tag_recognizer')
        self.cv_bridge = CvBridge()
        self.tag_detector = apriltag.Detector()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.base_apriltag_id = 99  # ID of the Apriltag representing the charging base
        # Define control constants
        self.linear_speed = 0.2
        self.angular_speed = 0.1
        self.target_distance = 0.3  # Distance from the base marker where the robot should stop
        # Flag to indicate whether the base marker has been found
        self.base_apriltag_found = False

    def image_callback(self, msg):
        if not self.base_apriltag_found:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            detections = self.tag_detector.detect(gray_image)
            for detection in detections:
                if detection.tag_id == self.base_apriltag_id:
                    self.base_apriltag_found = True
                    break
            if not self.base_apriltag_found:
                # Rotate the robot to continue the search
                cmd_vel_msg = Twist()
                cmd_vel_msg.angular.z = self.angular_speed
                self.pub_cmd_vel.publish(cmd_vel_msg)
        else:
            # Calculate the distance from the robot to the base tag
            distance = self.calculate_distance_to_base(msg)
            print(distance)
            if distance is not None:
                # If the distance is less than the target distance, stop the robot
                if distance < self.target_distance:
                    self.stop_robot()
                else:
                    # Move the robot forward
                    self.move_forward()
            else:
                self.get_logger().error("Error calculating distance to base.")

    def calculate_distance_to_base(self, msg):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        detections = self.tag_detector.detect(gray_image)
        for detection in detections:
            if detection.tag_id == self.base_apriltag_id:
                # Assuming the size of the tag is 0.1 meters
                tag_size = 0.1
                # Calculate the distance from the robot to the base tag
                # Assuming focal length of the camera is 300 pixels
                focal_length = 300
                distance = (tag_size * focal_length)
                return distance
        return None


    def stop_robot(self):
        # Stop the robot
        cmd_vel_msg = Twist()
        self.pub_cmd_vel.publish(cmd_vel_msg)

    def move_forward(self):
        # Move the robot forward
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = self.linear_speed
        self.pub_cmd_vel.publish(cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TagRecognizerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
