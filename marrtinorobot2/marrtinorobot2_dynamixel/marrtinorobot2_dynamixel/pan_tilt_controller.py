# Copyright 2025 robotics-3d.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Ferrarini Fabio
# Email : ferrarini09@gmail.com
# File  : pantilt_controller.py
#
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from dynamixel_sdk import *  # Libreria Dynamixel SDK
from std_msgs.msg import Float64  # Messaggio per i comandi di posizione

class DynamixelController(Node):
    def __init__(self):
        super().__init__('dynamixel_controller')

        # Inizializzazione della porta e del packet handler per Dynamixel
        self.port_handler = PortHandler('/dev/dynamixel')  # Porta seriale
        self.packet_handler = PacketHandler(1.0)         # Versione protocollo
        self.baudrate = 1000000
        self.pan_motor_id = 2
        self.tilt_motor_id = 1
        self.right_arm_motor_id = 3  # ID motore braccio destro
        self.left_arm_motor_id = 4   # ID motore braccio sinistro
        self.init_dynamixel()

        # Sottoscrittori per i comandi pan, tilt, braccio destro e sinistro
        self.pan_subscriber = self.create_subscription(
            Float64, 'pan_controller/command', self.pan_callback, 10)
        self.tilt_subscriber = self.create_subscription(
            Float64, 'tilt_controller/command', self.tilt_callback, 10)
        self.right_arm_subscriber = self.create_subscription(
            Float64, 'right_arm_controller/command', self.right_arm_callback, 10)
        self.left_arm_subscriber = self.create_subscription(
            Float64, 'left_arm_controller/command', self.left_arm_callback, 10)

    def init_dynamixel(self):
        if self.port_handler.openPort():
            self.get_logger().info('Port opened successfully.')
        else:
            self.get_logger().error('Failed to open port.')

        if self.port_handler.setBaudRate(self.baudrate):
            self.get_logger().info('Baudrate set successfully.')
        else:
            self.get_logger().error('Failed to set baudrate.')

    def degrees_to_position(self, degrees):
        """Converte i gradi in posizione Dynamixel (0-1023)."""
        return int(degrees * 1023 / 300)

    def position_to_degrees(self, position):
        """Converte la posizione Dynamixel (0-1023) in gradi."""
        return position * 300 / 1023

    def reset_servo(self, motor_id):
        result, error = self.packet_handler.factoryReset(self.port_handler, motor_id, 0xFF)
        if result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to reset motor {motor_id}: {self.packet_handler.getTxRxResult(result)}')
        elif error != 0:
            self.get_logger().error(f'Error occurred when resetting motor {motor_id}: {self.packet_handler.getRxPacketError(error)}')
        else:
            self.get_logger().info(f'Motor {motor_id} has been reset to factory defaults.')


    def set_max_torque(self, motor_id, torque_value=1023):
        result, error = self.packet_handler.write2ByteTxRx(self.port_handler, motor_id, 14, torque_value)
        if result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to set max torque: {self.packet_handler.getTxRxResult(result)}')
        elif error != 0:
            self.get_logger().error(f'Error occurred when setting max torque: {self.packet_handler.getRxPacketError(error)}')
        else:
            self.get_logger().info(f'Motor {motor_id} max torque set to {torque_value}')

    def enable_torque(self, motor_id):
        result, error = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, 24, 1)
        if result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to enable torque: {self.packet_handler.getTxRxResult(result)}')
        elif error != 0:
            self.get_logger().error(f'Error occurred when enabling torque: {self.packet_handler.getRxPacketError(error)}')
        else:
            self.get_logger().info(f'Motor {motor_id} torque enabled.')

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
        result_pos, error_pos = self.packet_handler.write2ByteTxRx(self.port_handler, motor_id, 30, int(position))
        if result_pos != COMM_SUCCESS:
            self.get_logger().error(f'Failed to set position: {self.packet_handler.getTxRxResult(result_pos)}')
        elif error_pos != 0:
            self.get_logger().error(f'Error occurred when setting position: {self.packet_handler.getRxPacketError(error_pos)}')
        else:
            self.get_logger().info(f'Motor {motor_id} moved to position {int(position)}')

    # Callback per il comando pan
    def pan_callback(self, msg):
        self.get_logger().info(f'Received pan command (degrees): {msg.data}')
        degree = msg.data
 
        position =  512 + self.degrees_to_position(degree)
 
        pan_position = position
        if pan_position > 648:
            pan_position = 648
        if pan_position < 376:
            pan_position = 376
        speed = 50  # Imposta la velocità del motore Pan
        self.get_logger().info(f'pan position: {pan_position}')
        self.set_position(self.pan_motor_id, pan_position, speed)

    # Callback per il comando tilt
    def tilt_callback(self, msg):
        self.get_logger().info(f'Received tilt command (degrees): {msg.data}')
        degree = msg.data

        position =  512 - self.degrees_to_position(degree)
 
        tilt_position = position
        if tilt_position < 410:
            tilt_position = 410
        if tilt_position > 580:
            tilt_position = 580
        speed = 50  # Imposta la velocità del motore Tilt
        self.set_position(self.tilt_motor_id, tilt_position, speed)

    # Callback per il comando braccio destro
    def right_arm_callback(self, msg):
        self.get_logger().info(f'Received right arm command (degrees): {msg.data}')
        #
        right_arm_position = self.degrees_to_position(150 + msg.data )  # Converte i gradi in posizione
        speed = 50  # Imposta la velocità del motore braccio destro
        self.set_position(self.right_arm_motor_id, right_arm_position, speed)

    # Callback per il comando braccio sinistro
    def left_arm_callback(self, msg):
        self.get_logger().info(f'Received left arm command (degrees): {msg.data}')
        left_arm_position = self.degrees_to_position(150 - msg.data)  # Converte i gradi in posizione
        speed = 50  # Imposta la velocità del motore braccio sinistro
        self.set_position(self.left_arm_motor_id, left_arm_position, speed)

def main(args=None):
    rclpy.init(args=args)

    # Inizializza il nodo DynamixelController
    controller = DynamixelController()
    # Configurazione dei motori: id, posizione iniziale, velocità
    # motor_configs = [
    #     {'id': 1, 'position_deg': 140, 'speed': 40},  # Tilt
    #     {'id': 2, 'position_deg': 150, 'speed': 40},  # Pan
    #     {'id': 3, 'position_deg': 150, 'speed': 40},  # Braccio destro
    #     {'id': 4, 'position_deg': 150, 'speed': 40},  # Braccio sinistro
    # ]

    # # Ciclo per inizializzare ogni motore
    # for motor in motor_configs:
    #     motor_id = motor['id']
    #     position = controller.degrees_to_position(motor['position_deg'])
    #     speed = motor['speed']

    #     controller.set_max_torque(motor_id, 1023)
    #     controller.enable_torque(motor_id)
    #     controller.set_position(motor_id, position, speed)

    # Imposta il motore pan a posizione 150 gradi e velocità 40
    position =  512 - controller.degrees_to_position(30)
    controller.set_position(1, controller.degrees_to_position(140), 40)
    #controller.set_position(1, controller.degrees_to_position(140), 40)
    # Imposta il motore tilt a posizione 150 gradi e velocità 40

    controller.set_position(2, controller.degrees_to_position(150), 40)
    # Imposta il motore braccio destro a posizione 150 gradi e velocità 40
    controller.set_position(3, controller.degrees_to_position(150), 40)
    # Imposta il motore braccio sinistro a posizione 150 gradi e velocità 40
    controller.set_position(4, controller.degrees_to_position(150), 40)

    # Esegui il nodo in modalità loop
    rclpy.spin(controller)

    # Spegni ROS 2 quando il nodo viene terminato
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
