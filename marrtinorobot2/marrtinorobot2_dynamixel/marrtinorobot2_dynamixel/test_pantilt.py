#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from dynamixel_sdk import *  # Libreria Dynamixel SDK


class DynamixelController(Node):
    def __init__(self):
        super().__init__('dynamixel_controller')
        self.port_handler = PortHandler('/dev/dynamixel')  # Porta seriale
        self.packet_handler = PacketHandler(1.0)         # Versione protocollo
        self.baudrate = 1000000
        self.pan_motor_id = 1
        self.tilt_motor_id = 2
        
        self.init_dynamixel()

    def init_dynamixel(self):
        if self.port_handler.openPort():
            self.get_logger().info('Port opened successfully.')
        else:
            self.get_logger().error('Failed to open port.')

        if self.port_handler.setBaudRate(self.baudrate):
            self.get_logger().info('Baudrate set successfully.')
        else:
            self.get_logger().error('Failed to set baudrate.')

    def set_position(self, motor_id, position, speed):
        # Imposta la velocità di movimento (indirizzo 32)
        result_speed, error_speed = self.packet_handler.write2ByteTxRx(self.port_handler, motor_id, 32, speed)
        if result_speed != COMM_SUCCESS:
            self.get_logger().error(f'Failed to set speed: {self.packet_handler.getTxRxResult(result_speed)}')
        elif error_speed != 0:
            self.get_logger().error(f'Error occurred when setting speed: {self.packet_handler.getRxPacketError(error_speed)}')
        else:
            self.get_logger().info(f'Motor {motor_id} speed set to {speed}')

        # Imposta la posizione del servo (indirizzo 30)
        result_pos, error_pos = self.packet_handler.write2ByteTxRx(self.port_handler, motor_id, 30, position)
        if result_pos != COMM_SUCCESS:
            self.get_logger().error(f'Failed to set position: {self.packet_handler.getTxRxResult(result_pos)}')
        elif error_pos != 0:
            self.get_logger().error(f'Error occurred when setting position: {self.packet_handler.getRxPacketError(error_pos)}')
        else:
            self.get_logger().info(f'Motor {motor_id} moved to position {position}')

def main(args=None):
    rclpy.init(args=args)
    controller = DynamixelController()
    
    # Imposta il motore pan a posizione 512 (centrale) e velocità 100
    controller.set_position(1, 512,40)
    # Imposta il motore tilt a posizione 512 (centrale) e velocità 200
    controller.set_position(2, 512, 40)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
