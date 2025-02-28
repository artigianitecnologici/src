import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import apriltag

class SearchTagNode(Node):
    def __init__(self):
        super().__init__('search_tag_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.cv_bridge = CvBridge()
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.desired_distance = 1.0  # Distanza desiderata dal tag
        self.k_linear = 0.1  # Costante di proporzionalità per il controllo lineare
        self.k_angular = 0.1  # Costante di proporzionalità per il controllo angolare

    def image_callback(self, msg):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Rileva i tag Apriltag nell'immagine utilizzando apriltag
        tag_center, tag_orientation = self.detect_tag(cv_image)

        if tag_center is not None:
            # Calcola l'errore di distanza dal tag
            error_distance = self.desired_distance - tag_center[2]
            
            # Calcola il comando di velocità lineare proporzionale all'errore di distanza
            linear_x = self.k_linear * error_distance
            
            # Calcola il comando di velocità angolare proporzionale all'angolo di orientazione del tag
            angular_z = self.k_angular * tag_orientation[2]
            
            # Pubblica il comando di velocità
            self.publish_velocity(linear_x, angular_z)
        else:
            # Se il tag non è trovato, arresta il movimento
            self.publish_velocity(0.0, 0.0)

    def detect_tag(self, image):
        # Converti l'immagine in scala di grigi
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Crea un oggetto apriltag.Detector
        detector = apriltag.Detector()
        
        # Rileva i tag Apriltag nell'immagine
        detections, _ = detector.detect(gray_image)
        
        if detections:
            # Se vengono rilevati tag, prendi il primo tag rilevato
            detection = detections[0]
            
            # Estrai il centro e l'orientazione del tag
            center = detection.center
            orientation = detection.homography[0:2, 0:2]
            
            return center, orientation
        else:
            # Se il tag non è trovato, restituisci None
            return None, None

    def publish_velocity(self, linear_x, angular_z):
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self.pub_cmd_vel.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    search_tag_node = SearchTagNode()
    search_tag_node.get_logger().info('Start find tag')
    rclpy.spin(search_tag_node)
    search_tag_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
