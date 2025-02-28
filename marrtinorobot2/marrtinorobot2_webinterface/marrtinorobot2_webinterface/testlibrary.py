#!/usr/bin/env python3

import rclpy
from robot_cmd_ros import RobotCmdROS  # Importa la libreria

def main():
    rclpy.init()
    robot = RobotCmdROS()

    try:
        robot.begin()
        print('Avanti')
        robot.forward(1.0)   # Move forward by 1 meter
        print('Indietro')
        robot.backward(1.0)  # Move backward by 1 meter
        print('Sinistra')
        robot.left(90)       # Rotate left by 90 degrees
        print('Destra')
        robot.right(90)      # Rotate right by 90 degrees
        robot.end()
    except KeyboardInterrupt:
        robot.get_logger().info('Interrupted by user')
        robot.end()
    finally:
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
