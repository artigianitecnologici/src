import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
# importa le librerie necessarie per il riconoscimento dei tag
from cv_bridge import CvBridge
import apriltag
import cv2

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
        self.base_apriltag_id = 99  # ID of the ArUco marker representing the charging base
        # Define control constants
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.target_distance = 1.0  # Distance from the base marker where the robot should stop
        # Flag to indicate whether the base marker has been found
        self.base_apriltag_found = False
         

    def image_callback(self, msg):
        # Converti l'immagine ROS in un'immagine OpenCV
        # e riconosci i tag
        if not self.base_apriltag_found:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            detections = self.tag_detector.detect(gray_image)
            idtag=0
            for detection in detections:
                idtag= detection.tag_id
                print("Tag ID:", idtag)
            if (idtag == self.base_apriltag_id):
                print("ok")
                self.base_apriltag_found = True
            else:
                # Rotate the robot to continue the search
                cmd_vel_msg = Twist()
                cmd_vel_msg.angular.z = self.angular_speed
                self.pub_cmd_vel.publish(cmd_vel_msg)
        else:
            # navigate to base 
            print("Trovato inizio avvicinamento")


def main(args=None):
    rclpy.init(args=args)
    node = TagRecognizerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
