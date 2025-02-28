import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudToLaserScan(Node):
    def __init__(self):
        super().__init__('pointcloud_to_laserscan')

        # QoS Profile for "Best Effort"
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # Sottoscrittore PointCloud2
        self.pointcloud_subscriber = self.create_subscription(
            PointCloud2,
            '/stereo/points',
            self.pointcloud_callback,
            qos_profile
        )

        # Publisher LaserScan
        self.scan_publisher = self.create_publisher(
            LaserScan,
            '/scan',
            qos_profile
        )

        # Parameters for LaserScan conversion
        self.angle_min = -1.57  # -90 degrees
        self.angle_max = 1.57   # +90 degrees
        self.range_min = 0.2    # Minimum range in meters
        self.range_max = 10.0   # Maximum range in meters
        self.angle_increment = 0.01  # Angular resolution in radians

    def pointcloud_callback(self, msg):
        # Parse PointCloud2 data
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points_array = np.array(list(points))

        # Filter points within a certain height range
        z_min, z_max = -0.1, 0.1
        filtered_points = [p for p in points_array if z_min <= p[2] <= z_max]

        # Initialize LaserScan message
        scan = LaserScan()
        scan.header = msg.header
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        # Number of beams in the LaserScan
        num_beams = int((self.angle_max - self.angle_min) / self.angle_increment)
        ranges = [float('inf')] * num_beams

        # Process filtered points to simulate LaserScan
        for point in filtered_points:
            x, y, z = point

            # Ignore points outside the valid range
            distance = np.sqrt(x**2 + y**2)
            if not self.range_min <= distance <= self.range_max:
                continue

            # Calculate angle
            angle = np.arctan2(y, x)

            # Ignore points outside the angular range
            if not self.angle_min <= angle <= self.angle_max:
                continue

            # Calculate index in LaserScan array
            index = int((angle - self.angle_min) / self.angle_increment)

            # Update range if this point is closer
            if 0 <= index < num_beams:
                ranges[index] = min(ranges[index], distance)

        # Ensure ranges are a valid sequence of floats
        ranges = [r if r != float('inf') else self.range_max + 1.0 for r in ranges]
        scan.ranges = ranges

        # Publish LaserScan message
        self.scan_publisher.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToLaserScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
