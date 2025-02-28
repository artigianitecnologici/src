import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from apriltag import Detector
import cv2
import numpy as np
import math

class TagRecognizerNode(Node):
    def __init__(self):
        super().__init__('tag_recognizer')
        self.cv_bridge = CvBridge()
        self.tag_detector = Detector()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.base_apriltag_id = 99  # ID of the Apriltag representing the charging base
        # Define control constants
        self.linear_speed = 0.4
        self.angular_speed = 0.2
        self.target_distance = 0.3  # Distance from the base marker where the robot should stop
        # Flag to indicate whether the base marker has been found
        self.base_apriltag_found = False

    def image_callback(self, msg):
        if not self.base_apriltag_found:
            # find the tag 99
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
            # apriltag found
            # Calculate the distance from the robot to the base tag
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            twist_msg = self.calculate_parking_movement(cv_image)
            self.pub_cmd_vel.publish(twist_msg)

    def calculate_parking_movement(self, cv_image):
        twist_msg = Twist()
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        detections = self.tag_detector.detect(gray)
        
        if detections:
            print ("tag rilevato")
            # Prendi solo la prima rilevazione (nell'ipotesi che ci sia solo una tag)
            detection = detections[0]
            # Calcola il centroide del tag
            centroid_x = int(detection.center[0])
            centroid_y = int(detection.center[1])
            # Calcola il punto centrale dell'immagine
            center_x = cv_image.shape[1] // 2
            center_y = cv_image.shape[0] // 2
            
            # Calcola l'errore rispetto al centro dell'immagine
            error_x = center_x - centroid_x
            error_y = center_y - centroid_y
            
            # Calcola l'angolo tra il centro dell'immagine e il tag
            angle = math.atan2(error_y, error_x)
            print(angle)
            # Calcola il movimento lineare e angolare in base all'angolo
            twist_msg.linear.x = self.linear_speed
            twist_msg.angular.z = self.angular_speed * angle
            
            # Se il tag è abbastanza vicino al centro, muovi avanti
            if abs(error_x) < 20 and abs(error_y) < 20:
                twist_msg.linear.x = self.linear_speed
                twist_msg.angular.z = 0.0
            
        return twist_msg

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
