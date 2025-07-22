#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarFilter180Node(Node):
    def __init__(self):
        super().__init__('lidar_filter_180_node')
        self.get_logger().info("Nodo filtro 180° avviato")

        # Abbonati al topic del LiDAR
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_unfiltered',  # Cambia questo se il tuo topic è diverso
            self.scan_callback,
            10)

        # Pubblica i dati filtrati su un nuovo topic
        self.publisher = self.create_publisher(
            LaserScan,
            '/scan',
            10)

        # Parametri del filtro
        self.min_range = 0.15
        self.max_range = 6.0
        self.min_angle = -np.pi / 2  # -90 gradi
        self.max_angle =  np.pi / 2  # +90 gradi

    def scan_callback(self, msg):
        #self.get_logger().info("Ricevuto messaggio dal LiDAR")

        # Copia il messaggio originale
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max

        # Lista dei range e degli angoli corrispondenti
        ranges = list(msg.ranges)
        angles = [msg.angle_min + i * msg.angle_increment for i in range(len(ranges))]

        for i, angle in enumerate(angles):
            # Rendi infinito se fuori dai 180° frontali o fuori range valido
            if angle < self.min_angle or angle > self.max_angle:
                ranges[i] = float('inf')
            elif ranges[i] < self.min_range or ranges[i] > self.max_range:
                ranges[i] = float('inf')

        filtered_msg.ranges = ranges
        filtered_msg.intensities = list(msg.intensities) if msg.intensities else []

        self.publisher.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarFilter180Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
