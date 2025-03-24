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
        self.right_shoulder_flexion_motor_id = 4
        self.left_shoulder_flexion_motor_id = 7
        self.right_shoulder_rotation_motor_id = 3
        self.left_shoulder_rotation_motor_id = 6       
        self.right_elbow_motor_id = 5
        self.left_elbow_motor_id = 8
        self.hand_right_motor_id = 10  
        self.hand_left_motor_id = 9   

        self.init_dynamixel()

        # Sottoscrittori per i comandi pan, tilt, braccio destro e sinistro
        self.pan_subscriber = self.create_subscription(
            Float64, 'pan_controller/command', self.pan_callback, 10)
        self.tilt_subscriber = self.create_subscription(
            Float64, 'tilt_controller/command', self.tilt_callback, 10)
        self.right_shoulder_flexion_subscriber = self.create_subscription(
            Float64, 'right_shoulder_flexion_controller/command', self.right_shoulder_flexion_callback, 10)
        self.left_shoulder_flexion_subscriber = self.create_subscription(
            Float64, 'left_shoulder_flexion_controller/command', self.left_shoulder_flexion_callback, 10)
        self.right_shoulder_rotation_subscriber = self.create_subscription(
            Float64, 'right_shoulder_rotation_controller/command', self.right_shoulder_rotation_callback, 10)
        self.left_shoulder_rotation_subscriber = self.create_subscription(
            Float64, 'left_shoulder_rotation_controller/command', self.left_shoulder_rotation_callback, 10)
        self.right_elbow_subscriber = self.create_subscription(
            Float64, 'right_elbow_controller/command', self.right_elbow_callback, 10)
        self.left_shoulder_rotation_subscriber = self.create_subscription(
            Float64, 'left_elbow_controller/command', self.left_elbow_callback, 10)
        self.hand_right_subscriber = self.create_subscription(
            Float64, 'hand_right_controller/command', self.hand_right_callback, 10)
        self.hand_left_subscriber = self.create_subscription(
            Float64, 'hand_left_controller/command', self.hand_left_callback, 10)


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

    # Callback per il comando pan   limiti 50 e 50
    def pan_callback(self, msg):
        self.get_logger().info(f'Received pan command (degrees): {msg.data}')
        degree = msg.data
 #       if degree > 0 :
        position =  512 + self.degrees_to_position(degree)
 #       else:
 #           position =  512 + self.degrees_to_position(-degree)
        pan_position = position 
        speed = 50  # Imposta la velocità del motore Pan
        self.get_logger().info(f'pan position: {pan_position}')
        self.set_position(self.pan_motor_id, pan_position, speed)

    # Callback per il comando tilt   +20 -20 limiti
    def tilt_callback(self, msg):
        self.get_logger().info(f'Received tilt command (degrees): {msg.data}')
        degree = msg.data
 #       if degree > 0 :
        position =  512 - self.degrees_to_position(degree)
 #       else:
 #           position =  512 + self.degrees_to_position(-degree)
        tilt_position = position 
        speed = 50  # Imposta la velocità del motore Tilt
        self.set_position(self.tilt_motor_id, tilt_position, speed)

    # Callback per il comando spalla rotazione destra
    def right_shoulder_rotation_callback(self, msg):
        self.get_logger().info(f'Received right shoulder rotation command (degrees): {msg.data}')
        degree = msg.data
 #       if degree > 0 :
        position =  512 + self.degrees_to_position(degree)
 #       else:
 #           position =  512 + self.degrees_to_position(-degree)
         
        right_shoulder_rotation_position = position
        speed = 50  # Imposta la velocità del motore braccio destro
        self.set_position(self.right_shoulder_rotation_motor_id, right_shoulder_rotation_position, speed)

    # Callback per il comando spalla rotazione sinistro
    def left_shoulder_rotation_callback(self, msg):
        self.get_logger().info(f'Received left shoulder rotation command (degrees): {msg.data}')
        degree = msg.data
 #       if degree > 0 :
        position =  512 - self.degrees_to_position(degree)
 #       else: 
  #           position =  512 + self.degrees_to_position(degree)
         
        left_shoulder_rotation_position = position
        
        speed = 50  # Imposta la velocità del motore braccio sinistro
        self.set_position(self.left_shoulder_rotation_motor_id, left_shoulder_rotation_position, speed)

 # Callback per il comando spalla flessione destra
    def right_shoulder_flexion_callback(self, msg):
        self.get_logger().info(f'Received right shoulder flexion command (degrees): {msg.data}')
        degree = msg.data
 #       if degree > 0 :
        position =  512 - self.degrees_to_position(degree)
 #        else:
 #           position =  512 + self.degrees_to_position(-degree)
         
        right_shoulder_flexion_position = position
        
        speed = 50  # Imposta la velocità del motore braccio destro
        self.set_position(self.right_shoulder_flexion_motor_id, right_shoulder_flexion_position, speed)

    # Callback per il comando spalla flessione sinistro
    def left_shoulder_flexion_callback(self, msg):
        self.get_logger().info(f'Received left shoulder flexion command (degrees): {msg.data}')
        degree = msg.data
 #       if degree > 0 :
        position =  512 + self.degrees_to_position(degree)
 #       else:
 #           position =  512 + self.degrees_to_position(degree)
         
        left_shoulder_flexion_position = position
        
        speed = 50  # Imposta la velocità del motore braccio sinistro
        self.set_position(self.left_shoulder_flexion_motor_id, left_shoulder_flexion_position, speed)

    # Callback per il comando gomito destro  limiti 0 90
    def right_elbow_callback(self, msg):
        self.get_logger().info(f'Received right elbow command (degrees): {msg.data}')
        degree = msg.data
 #       if degree > 0 :
        position =  512 + self.degrees_to_position(degree)
 #       else:
 #           position =  512 + self.degrees_to_position(-degree)
         
        right_elbow_position  = position
       
        speed = 50  # Imposta la velocità del motore braccio destro
        self.set_position(self.right_elbow_motor_id, right_elbow_position, speed)

    # Callback per il comando gomito sinistro  limiti 0 90
    def left_elbow_callback(self, msg):
        self.get_logger().info(f'Received left elbow command (degrees): {msg.data}')
        degree = msg.data
 #       if degree > 0 :
        position =  512 - self.degrees_to_position(degree)
 #       else:
 #           position =  512 + self.degrees_to_position(degree)
         
        
        left_elbow_position = position
        speed = 50  # Imposta la velocità del motore braccio destro
        self.set_position(self.left_elbow_motor_id, left_elbow_position, speed)


    # Callback per il comando mano destra
    def hand_right_callback(self, msg):
        self.get_logger().info(f'Received hand right command (degrees): {msg.data}')
        degree = msg.data
 #       if degree > 0 :
        position =  512 + self.degrees_to_position(degree)
 #       else:
 #           position =  512 + self.degrees_to_position(-degree)
         
        
         
        hand_right_position = position
        speed = 50  # Imposta la velocità del motore braccio sinistro
        self.set_position(self.hand_right_motor_id, hand_right_position, speed)
    
    # Callback per il comando mano sinistra
    def hand_left_callback(self, msg):
        self.get_logger().info(f'Received hand left command (degrees): {msg.data}')
        degree = msg.data
 #       if degree > 0 :
        position =  512 - self.degrees_to_position(degree)
 #       else:
 #           position =  512 + self.degrees_to_position(degree)
         
        
         
        hand_left_position = position
        speed = 50  # Imposta la velocità del motore braccio sinistro
        self.set_position(self.hand_left_motor_id, hand_left_position, speed)


def main(args=None):
    rclpy.init(args=args)

    # Inizializza il nodo DynamixelController
    controller = DynamixelController()
     # Imposta il motore pan a posizione 512 gradi e velocità 40
    controller.set_position(1, controller.degrees_to_position(150), 40)
    # Imposta il motore tilt a posizione 512 gradi e velocità 40
    controller.set_position(2, controller.degrees_to_position(150), 40)
    # Imposta il motore rotazione destro a posizione 512 gradi e velocità 40
    controller.set_position(3, controller.degrees_to_position(150), 40)
    # Imposta il motore rotazione sinistro a posizione 512 gradi e velocità 40
    controller.set_position(4, controller.degrees_to_position(150), 40)
    # Imposta il motore flessione destra a posizione 512 gradi e velocità 40
    controller.set_position(5, controller.degrees_to_position(150), 40)
    # Imposta il motore flessione sinistra a posizione 512 gradi e velocità 40
    controller.set_position(6, controller.degrees_to_position(150), 40)
    # Imposta il motore gomito destro a posizione 512 gradi e velocità 40
    controller.set_position(7, controller.degrees_to_position(150), 40)
    # Imposta il motore gomito sinistro a posizione 512 gradi e velocità 40
    controller.set_position(8, controller.degrees_to_position(150), 40)
    # Imposta il motore mano destra a  posizione 512 gradi e velocità 40
    controller.set_position(9, controller.degrees_to_position(150), 40)
    # Imposta il motore mano sinistra a posizione 512 gradi e velocità 40
    controller.set_position(10, controller.degrees_to_position(150), 40)



    # Esegui il nodo in modalità loop
    rclpy.spin(controller)

    # Spegni ROS 2 quando il nodo viene terminato
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# ros2 topic pub /pan_motor_command std_msgs/msg/Float64 "data: 45.0"
# ros2 topic pub /tilt_motor_command std_msgs/msg/Float64 "data: 30.0"