import rclpy
from robot_cmd_ros import RobotCmdROS

def main():
    rclpy.init()
    robot = RobotCmdROS()

    robot.begin()
    robot.right(20)  # 🔁 ESEMPIO: ruota di 90 gradi a destra
    robot.end() 
    robot.stop_thread()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
