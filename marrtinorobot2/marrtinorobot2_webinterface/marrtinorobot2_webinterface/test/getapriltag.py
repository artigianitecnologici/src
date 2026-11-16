import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray

def tag_callback(msg):
    if msg.detections:
        for detection in msg.detections:
            tag_id = detection.id
            tag_size = detection.size
            position = detection.pose.pose.pose.position
            orientation = detection.pose.pose.pose.orientation
            
            print(f"Tag ID: {tag_id}")
            print(f"Size: {tag_size}")
            print(f"Position: x={position.x}, y={position.y}, z={position.z}")
            print(f"Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}")
    else:
        print("No AprilTag detected.")

class AprilTagSubscriber(Node):
    def __init__(self):
        super().__init__('apriltag_listener')
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag_detections',
            tag_callback,
            10  # QoS depth
        )
        self.subscription  # prevent unused variable warning

def main():
    rclpy.init()
    node = AprilTagSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
