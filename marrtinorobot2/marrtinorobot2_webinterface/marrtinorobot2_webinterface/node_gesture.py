#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
import math
import time

class GestureNode(Node):
    def __init__(self):
        super().__init__('gesture_node')

        # Topic definitions
        self.TOPIC_emotion = "social/emotion"
        self.TOPIC_speech = "speech/to_speak"
        self.TOPIC_pan = "pan_controller/command"
        self.TOPIC_tilt = "tilt_controller/command"
        self.TOPIC_right_arm = "right_arm_controller/command"
        self.TOPIC_left_arm = "left_arm_controller/command"
        self.TOPIC_gesture = "/social/gesture"


        # Publisher definitions
        self.emotion_pub = self.create_publisher(String, self.TOPIC_emotion, 10)
        self.speech_pub = self.create_publisher(String, self.TOPIC_speech, 10)
        self.pan_pub = self.create_publisher(Float64, self.TOPIC_pan, 10)
        self.tilt_pub = self.create_publisher(Float64, self.TOPIC_tilt, 10)
        self.left_arm_pub = self.create_publisher(Float64, self.TOPIC_left_arm, 10)
        self.right_arm_pub = self.create_publisher(Float64, self.TOPIC_right_arm, 10)

        # Declare parameters
        self.declare_parameter("face_reset_timer", 5)
        self.TIME_DELAY = self.get_parameter("face_reset_timer").value

        # Timer setup
        self.timer = self.create_timer(self.TIME_DELAY, self.reset_gesture)

        # Subscriber
        self.create_subscription(String, self.TOPIC_gesture, self.callback_gesture, 10)
        
        self.get_logger().info("Gesture node started")

    def say(self, msg):
        self.get_logger().info(f'speech: {msg}')
        message = String()
        message.data = msg
        self.speech_pub.publish(message)

    def emotion(self, msg):
        self.get_logger().info(f'social/emotion: {msg}')
        message = String()
        message.data = msg
        self.emotion_pub.publish(message)

    def pan(self, msg):
        self.get_logger().info(f'Pan Position: {msg}')
        message = Float64()
        message.data = msg
        self.pan_pub.publish(message)

    def tilt(self, msg):
        self.get_logger().info(f'Tilt Position: {msg}')
        message = Float64()
        message.data = msg
        self.tilt_pub.publish(message)

    def left_arm(self, msg):
        self.get_logger().info(f'Left Arm Position: {msg}')
        message = Float64()
        message.data = msg
        self.left_arm_pub.publish(message)

    def right_arm(self, msg):
        self.get_logger().info(f'right Arm Position: {msg}')
        message = Float64()
        message.data = msg
        self.right_arm_pub.publish(message)

    def head_position(self, msg):
        self.get_logger().info(f'Head Position: {msg}')
        if msg == 'front':
            self.pan(0)
            self.tilt(0)
        elif msg == 'left':
            self.pan(30)
            self.tilt(0)
        elif msg == 'right':
            self.pan(-30)
            self.tilt(0)
        elif msg == 'up':
            self.pan(0)
            self.tilt(-30)
        elif msg == 'down':
            self.pan(0)
            self.tilt(30)

    def reset_gesture(self):
        self.get_logger().info("Resetting gesture animation")

    def gesture_init(self):
        self.head_position('front')
        self.left_arm(30)
        self.right_arm(-30)

    def gesture_zero(self):
        self.head_position('front')
        self.left_arm(0)
        self.right_arm(0)


    def callback_gesture(self, data):
        gesture = data.data
        self.get_logger().info(f'Received gesture: {gesture}')

        if gesture == 'init':
            self.gesture_init()
        elif gesture == 'gesture':
            self.gesture_anim()
        elif gesture == 'zero':
            self.gesture_zero()
        elif gesture == 'down':
            self.gesture_down()
        elif gesture == 'up':
            self.gesture_up()
        elif gesture == 'start':
            self.start_timer()
        elif gesture == 'hello':
            self.hello()
        elif gesture == 'indica_sinistra':
            self.gesture_indica_sinistra()
        elif gesture == 'point_left':
            self.point_left()
        elif gesture == 'point_right':
            self.point_right()
        elif gesture == 'point_up':
            self.point_up()
        elif gesture == 'greeting':
            self.greeting()
        elif gesture == 'check_ticket':
            self.check_ticket()

    # Define other gesture methods as needed (e.g., gesture_init, gesture_anim, etc.)

    def start_timer(self):
        self.get_logger().info("Starting timer")
        self.timer.reset()

    def stop_timer(self):
        self.get_logger().info("Stopping timer")
        self.timer.cancel()

    @staticmethod
    def DEG2RAD(a):
        return a * math.pi / 180.0

    @staticmethod
    def RAD2DEG(a):
        return a / math.pi * 180.0


def main(args=None):
    rclpy.init(args=args)
    gesture_node = GestureNode()
    rclpy.spin(gesture_node)
    gesture_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
