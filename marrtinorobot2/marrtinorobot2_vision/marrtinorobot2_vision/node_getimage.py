import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import time 
from datetime import datetime

class ImageGrabber(Node):
    def __init__(self):
        super().__init__('image_grabber')
        self.bridge = CvBridge()
        self.cvimage = None
        self.image_sub = self.create_subscription(
            Image, self.auto_image_topic(), self.image_cb, 10)
        self.shot_sub = self.create_subscription(
            String, 'getimage', self.shot_cb, 10)
        self.logdir = os.path.expanduser("~/playground/log/")
        if not os.path.isdir(self.logdir):
            self.logdir = os.path.expanduser("~/log/")
        if not os.path.isdir(self.logdir):
            self.logdir = "/tmp/"
        self.imagedir = os.path.expanduser("~/src/marrtinorobot2/marrtinorobot2_webinterface/www/viewer/img")
        self.get_logger().info("ImageGrabber node initialized.")

    def auto_image_topic(self):
        return '/camera/image_raw'

    def image_cb(self, data):
        #self.get_logger().info(f"Received an image: {data.header.stamp.sec}.{data.header.stamp.nanosec}")
        try:
            self.cvimage = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        #    self.get_logger().info("Image converted successfully!")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {str(e)}")

    def shot_cb(self, msg):
        self.get_logger().info(f"Shot command received: {msg.data}")
        self.get_image()

    def get_image(self, sleep_time=3):
        self.get_logger().info("Acquiring image...")
        time.sleep(sleep_time)
        if self.cvimage is None:
            self.get_logger().error("No image received!")
            return None
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        img_path = os.path.join(self.imagedir, "lastimage.jpg")
        cv2.imwrite(img_path, self.cvimage)
        self.get_logger().info(f"Image saved at {img_path}")
        return self.cvimage

def main(args=None):
    rclpy.init(args=args)
    node = ImageGrabber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
